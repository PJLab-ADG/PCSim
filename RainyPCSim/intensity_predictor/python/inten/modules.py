import os.path as osp

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F


class Fire(nn.Module):
    def __init__(self, in_channels, squeeze, expand, cam=False, top_parent=None):
        super().__init__()
        self.in_channels = in_channels
        self.out_channels = expand * 2
        self.squeeze = Conv(in_channels, squeeze, 1, top_parent=top_parent)
        self.expand1x1 = Conv(squeeze, expand, 1, top_parent=top_parent)
        self.expand3x3 = Conv(squeeze, expand, 3, 1, top_parent=top_parent)
        if cam:
            self.cam = ContextAggregation(self.out_channels, top_parent=top_parent)
        else:
            self.cam = None

    def forward(self, x):
        sq = self.squeeze(x)
        e1 = self.expand1x1(sq)
        e3 = self.expand3x3(sq)
        c = torch.cat([e1, e3], 1)
        if self.cam is not None:
            return self.cam(c)
        return c


class DeFire(nn.Module):
    def __init__(self, in_channels, squeeze, expand, cam=False, top_parent=None):
        super().__init__()
        self.in_channels = in_channels
        self.out_channels = expand * 2
        self.squeeze = Conv(in_channels, squeeze, 1, top_parent=top_parent)
        self.deconv = DeConv(squeeze)
        self.expand1x1 = Conv(squeeze, expand, 1, top_parent=top_parent)
        self.expand3x3 = Conv(squeeze, expand, 3, 1, top_parent=top_parent)
        if cam:
            self.cam = ContextAggregation(self.out_channels, top_parent=top_parent)
        else:
            self.cam = None

    def forward(self, x):
        sqd = self.deconv(self.squeeze(x))
        e1 = self.expand1x1(sqd)
        e3 = self.expand3x3(sqd)
        c = torch.cat([e1, e3], 1)
        if self.cam is not None:
            return self.cam(c)
        return c


class Pool(nn.Module):
    def __init__(self, size, stride, pad=0, top_parent=None):
        super().__init__()
        if top_parent is not None:
            top_parent.reduce *= stride
        self.pool = nn.MaxPool2d(size, (1, stride), padding=pad)

    def forward(self, x):
        return self.pool(x)


class ContextAggregation(nn.Module):
    def __init__(self, channels, reduction=16, top_parent=None):
        super().__init__()
        mid = channels // reduction
        self.in_channels = channels
        self.out_channels = channels
        nets = [
            Pool(7, 1, 3, top_parent=top_parent),
            Conv(channels, mid, 1, relu=True, norm=False, top_parent=top_parent),
            Conv(mid, channels, 1, relu=False, norm=False, top_parent=top_parent),
            torch.nn.Sigmoid(),
        ]
        self.nets = nn.Sequential(*nets)

    def forward(self, x):
        return x * self.nets(x)


class Conv(nn.Module):
    def __init__(self, in_channels, out_channels, kernel_size, pad=0, stride=1, relu=True, norm=True, top_parent=None):
        super().__init__()
        self.in_channels = in_channels
        self.out_channels = out_channels
        if top_parent is not None:
            top_parent.reduce *= stride
        nets = []
        nets.append(nn.Conv2d(in_channels, out_channels, kernel_size, padding=pad, stride=(1, stride)))
        if relu:
            nets.append(nn.ReLU(inplace=True))
        if norm:
            nets.append(nn.BatchNorm2d(out_channels))
        self.net = nn.Sequential(*nets)

    def forward(self, x):
        return self.net(x)


class DeConv(nn.Module):
    def __init__(self, channels, relu=True, norm=True):
        super().__init__()
        self.in_channels = channels
        self.out_channels = channels
        nets = []
        nets.append(nn.ConvTranspose2d(channels, channels, (1, 4), (1, 2), padding=(0, 1)))
        if relu:
            nets.append(nn.ReLU(inplace=True))
        if norm:
            nets.append(nn.BatchNorm2d(channels))
        self.net = nn.Sequential(*nets)

    def forward(self, x):
        return self.net(x)


class SqueezePartRaw(nn.Module):
    SQ_ADD = 16
    EF_ADD = 64

    def __init__(self, input_channels, sq, ef, depth, cam_depth=0, top_parent=None):
        super().__init__()
        efs = []
        sqs = []
        input_channelss = []
        cam = cam_depth > 0
        self.depth = depth
        self.down = nn.ModuleList()
        # self.net.append()
        self.pool = Pool(3, 2, 1, top_parent=top_parent)
        self.up = nn.ModuleList()
        for i in range(depth):
            self.down.append(nn.Sequential(Fire(input_channels, sq, ef, cam, top_parent=top_parent),
                                           Fire(2 * ef, sq, ef, cam, top_parent=top_parent)))
            sqs.append(sq)
            efs.append(ef)
            input_channelss.append(input_channels)
            input_channels = ef * 2
            ef += self.EF_ADD
            sq += self.SQ_ADD
        self.down.append(nn.Sequential(
            Fire(input_channels, sq, ef, cam, top_parent=top_parent),
            Fire(2 * ef, sq, ef, cam, top_parent=top_parent),
            Fire(2 * ef, sq + self.SQ_ADD, ef + self.EF_ADD, cam, top_parent=top_parent),
            Fire(2 * (ef + self.EF_ADD), sq + self.SQ_ADD, ef + self.EF_ADD, cam, top_parent=top_parent),
        ))
        for i in range(depth):
            sq = sqs.pop()
            ef = efs.pop()
            self.up.append(
                DeFire(2 * (ef + self.EF_ADD * (2 if i == 0 else 1)), 2 * sq, ef, top_parent=top_parent))

    def forward(self, x):
        res = []
        for i in range(self.depth):
            x = self.down[i](x)
            res.append(torch.clone(x))
            x = self.pool(x)
        x = self.down[self.depth](x)
        feat = torch.clone(x)
        for i in range(self.depth):
            x = self.up[i](x)
            x += res[self.depth - i - 1]
        return feat, x


class SqueezePart(nn.Module):
    SQ_ADD = 16
    EF_ADD = 64

    def __init__(self, input_channels, sq, ef, depth, cam_depth=0, top_parent=None):
        super().__init__()
        cam = cam_depth > 0
        if depth == 0:
            self.net = nn.Sequential(
                Fire(input_channels, sq, ef, cam, top_parent=top_parent),
                Fire(2 * ef, sq, ef, cam, top_parent=top_parent),
                Fire(2 * ef, sq + self.SQ_ADD, ef + self.EF_ADD, cam, top_parent=top_parent),
                Fire(2 * (ef + self.EF_ADD), sq + self.SQ_ADD, ef + self.EF_ADD, cam, top_parent=top_parent),
            )
        else:
            self.beg = nn.Sequential(Fire(input_channels, sq, ef, cam, top_parent=top_parent),
                                     Fire(2 * ef, sq, ef, cam, top_parent=top_parent))
            self.rest = nn.Sequential(
                Pool(3, 2, 1, top_parent=top_parent),
                SqueezePart(2 * ef, sq + self.SQ_ADD, ef + self.EF_ADD, depth - 1, cam_depth - 1,
                            top_parent=top_parent),
                DeFire(2 * (ef + self.EF_ADD * (2 if depth == 1 else 1)), 2 * sq, ef, top_parent=top_parent),
            )
        self.depth = depth

    def forward(self, x):
        if self.depth:
            pre_add = self.beg(x)
            insides = self.rest(pre_add)
            return pre_add + insides
        else:
            return self.net(x)


class XYZ(nn.Module):
    vert_angles = np.radians(
        np.concatenate((np.linspace(4 + (1.0 / 3), (-8 - 1.0 / 3), 40),
                        np.linspace((-8 - 1.0 / 3 - 1.0 / 2), (-24 - 1.0 / 3), 32)))
    )
    hor_angles = np.radians(np.flip(np.arange(0, 360, 0.1728)) + 180)
    ray = np.array([1.0, 0, 0])

    def __init__(self, dist_dim, mask_dim, x_start=0):
        super().__init__()
        self.dist_dim = dist_dim
        self.mask_dim = mask_dim
        self.ray = torch.from_numpy(self.ray)
        self.x_start = x_start
        self.vert_rotmat = torch.from_numpy(
            np.array([[[np.cos(angle), 0, -np.sin(angle)], [0, 1, 0], [np.sin(angle), 0, np.cos(angle)]] for angle in
                      self.vert_angles])
        )
        self.hor_rotmat = torch.from_numpy(
            np.array([[[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]] for angle in
                      self.hor_angles])
        )

    def forward(self, data):
        batch, y, x = torch.nonzero(data[:, self.mask_dim, ...] >= 0.5, as_tuple=True)
        matx = self.x_start + x
        pts = (self.hor_rotmat[matx, ...] @ self.vert_rotmat[y, ...] @ self.ray).float()
        dists = data[batch, self.dist_dim, y, x, None]
        b, _, ys, xs = data.shape
        result = torch.zeros(b, 4, ys, xs, dtype=data.dtype, device=data.device)
        result[batch, :3, y, x] = dists * pts
        result[:, 3, ...] = data[:, self.mask_dim, ...]
        return result


class CRF(nn.Module):
    SQ_VAR_BI = np.array([0.015, 0.015, 0.01, 0.01]) ** 2
    SQ_VAR_ANG = np.array([0.9, 0.9, 0.6, 0.6]) ** 2

    def __init__(
            self,
            num_iterations,
            bf_start_dim,
            bf_dims,
            mask_dim=-1,
            size_a=3,
            size_b=5,
            sq_var_bi=None,
            sq_var_ang=None,
            sq_var_bi_ang=None,
            ang_coef=0.02,
            bi_coef=0.1,
    ):
        super().__init__()
        if sq_var_ang is None:
            sq_var_ang = self.SQ_VAR_ANG
        if sq_var_bi is None:
            sq_var_bi = self.SQ_VAR_BI
        num_classes = len(sq_var_ang)
        init = (np.ones((num_classes, num_classes)) - np.eye(num_classes))[..., None, None].astype(np.float32)
        self.mask_dim = mask_dim
        self.bilateral = _BilateralWeights(size_a, size_b, bf_dims, sq_var_bi)
        self.local = _LocalPassing(size_a, size_b, num_classes, sq_var_ang, sq_var_bi_ang)
        self.ang_compat = nn.Conv2d(num_classes, num_classes, 1, bias=False)
        self.bi_ang_compat = nn.Conv2d(num_classes, num_classes, 1, bias=False)
        self.iterations = num_iterations
        self.bf_start_dim = bf_start_dim
        self.bf_dims = bf_dims
        self.ang_compat.weight = nn.Parameter(torch.from_numpy(init * ang_coef))
        self.bi_ang_compat.weight = nn.Parameter(torch.from_numpy(init * bi_coef))

    def forward(self, lidar_input, data):
        bf_weights = self.bilateral(lidar_input[:, self.bf_start_dim: self.bf_start_dim + self.bf_dims])
        mask = (lidar_input[:, self.mask_dim, None, ...] >= 0.5).float()
        for _ in range(self.iterations):
            unary = F.softmax(data, 1)
            ang, bi_ang = self.local(unary, mask, bf_weights)
            ang = self.ang_compat(ang)
            bi_ang = self.bi_ang_compat(bi_ang)
            outputs = unary + ang + bi_ang
            data = outputs
        return outputs


class DropoutNoise(nn.Module):
    def __init__(self, np_file=osp.join(osp.dirname(osp.abspath(__file__)), 'mask.npy')):
        super().__init__()
        self.mask = torch.from_numpy(np.load(np_file)).clamp(0, 1)[None, ...]

    def forward(self, data):
        bsize = data.shape[0]
        for i in range(bsize):
            mask = torch.bernoulli(self.mask).float()
            data[i] *= mask
        return data


class RGB2GS(nn.Module):
    _RGB2GS = np.array([0.2126, 0.7152, 0.0722], dtype=np.float32).reshape((1, 3, 1, 1))
    _GAMMA = 2.2
    _MULT = 116
    _EXP = 1 / 3
    _MINUS = 16
    _THRESH_CU = (6 / 29) ** 3
    _THRESH_SQ_THR = 3 * ((6 / 29) ** 2)
    _NORM = 100
    _ADD = 4 / 29

    def __init__(self, dim_start=0, as_tuple=False):
        super().__init__()
        self.dim_start = dim_start
        self.conv = nn.Conv2d(3, 1, 1, bias=False)
        self.conv.weight = nn.Parameter(torch.from_numpy(self._RGB2GS), requires_grad=False)
        self.as_tuple = as_tuple

    def forward(self, data):
        rgb = data[:, self.dim_start: self.dim_start + 3]
        rgb = rgb ** self._GAMMA
        gs = self.conv(rgb)
        mask = gs > self._THRESH_CU
        gs[mask] = gs[mask] ** self._EXP
        gs[~mask] = gs[~mask] / self._THRESH_SQ_THR + self._ADD
        gs = (self._MULT * gs - self._MINUS) / self._NORM
        data = torch.cat((data[:, : self.dim_start], gs, data[:, self.dim_start + 3:]), 1)
        if self.as_tuple:
            return (data,)
        return data


class _LocalPassing(nn.Module):
    def __init__(self, size_a, size_b, in_channels, sq_var_ang, sq_var_bi=None):
        if sq_var_bi is None:
            sq_var_bi = sq_var_ang
        pad = (size_a // 2, size_b // 2)
        super().__init__()
        self.ang_conv = nn.Conv2d(in_channels, in_channels, (size_a, size_b), padding=pad, bias=False)
        self.bi_ang_conv = nn.Conv2d(in_channels, in_channels, (size_a, size_b), padding=pad, bias=False)
        self.condense_conv = nn.Conv2d(in_channels, (size_a * size_b - 1) * in_channels, (size_a, size_b), padding=pad,
                                       bias=False)

        self.ang_conv.weight = nn.Parameter(torch.from_numpy(_gauss_weights(size_a, size_b, in_channels, sq_var_ang)),
                                            requires_grad=False)
        self.bi_ang_conv.weight = nn.Parameter(torch.from_numpy(_gauss_weights(size_a, size_b, in_channels, sq_var_bi)),
                                               requires_grad=False)
        self.condense_conv.weight = nn.Parameter(torch.from_numpy(_condensing_weights(size_a, size_b, in_channels)),
                                                 requires_grad=False)

    def forward(self, data, mask, bilateral):
        b, c, h, w = data.shape
        ang = self.ang_conv(data)
        bi_ang = self.bi_ang_conv(data)
        condense = self.condense_conv(data * mask).view(b, c, -1, h, w)
        bi_out = (condense * bilateral).sum(2) * mask * bi_ang
        return ang, bi_out


class _BilateralWeights(nn.Module):
    def __init__(self, size_a, size_b, in_channels, sq_var):
        super().__init__()
        pad = (size_a // 2, size_b // 2)
        self.in_channels = in_channels
        self.sq_var = sq_var
        self.condense_conv = nn.Conv2d(in_channels, (size_a * size_b - 1) * in_channels, (size_a, size_b), padding=pad,
                                       bias=False)
        self.condense_conv.weight = nn.Parameter(torch.from_numpy(_condensing_weights(size_a, size_b, in_channels)),
                                                 requires_grad=False)

    def forward(self, data):
        condensed = self.condense_conv(data)
        diffs = [data[:, i, None, ...] - condensed[:, i:: self.in_channels, ...] for i in range(self.in_channels)]
        return torch.stack(
            [torch.exp_(-sum([diff ** 2 for diff in diffs]) / (2 * self.sq_var[i])) for i in range(len(self.sq_var))],
            1)


def _gauss_weights(size_a, size_b, num_classes, sq_var):
    kernel = np.zeros((num_classes, num_classes, size_a, size_b), dtype=np.float32)
    for k in range(num_classes):
        kernel_2d = np.zeros((size_a, size_b), dtype=np.float32)
        for i in range(size_a):
            for j in range(size_b):
                diff = np.sum((np.array([i - size_a // 2, j - size_b // 2])) ** 2)
                kernel_2d[i, j] = np.exp(-diff / 2 / sq_var[k])
        kernel_2d[size_a // 2, size_b // 2] = 0
        kernel[k, k] = kernel_2d
    return kernel


def _condensing_weights(size_a, size_b, in_channels):
    half_filter_dim = (size_a * size_b) // 2
    kernel = np.zeros((size_a * size_b * in_channels, in_channels, size_a, size_b), dtype=np.float32)
    for i in range(size_a):
        for j in range(size_b):
            for k in range(in_channels):
                kernel[i * (size_b * in_channels) + j * in_channels + k, k, i, j] = 1
    kernel = np.concatenate([kernel[: in_channels * half_filter_dim], kernel[in_channels * (half_filter_dim + 1):]],
                            axis=0)
    return kernel
