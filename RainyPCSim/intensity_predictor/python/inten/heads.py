import torch
import torch.nn as nn
import torch.nn.functional as F

from . import modules as md


class TransformHead(nn.Module):
    def __init__(self, mid_size=512, dim=3, affine=True, coords=True, start_dim=0):
        super().__init__()
        self.coords = coords
        self.affine = affine
        self.dim = dim
        self.start_dim = start_dim
        self.ndim = dim + (1 if self.affine else 0)
        self.net = nn.Sequential(nn.AdaptiveAvgPool1d(mid_size), nn.ReLU(inplace=True),
                                 nn.Linear(mid_size, dim * self.ndim))
        self.net[-1].weight.data.zero_()
        self.net[-1].bias.data.copy_(torch.eye(self.dim, self.ndim).view(-1))

    def forward(self, data_input, features):
        trans = self.net(features.view(features.shape[0], 1, -1))
        trans = trans.view(-1, self.dim, self.ndim)
        if self.coords:
            grid = F.affine_grid(trans, data_input.size())
            result = F.grid_sample(data_input, grid, mode='nearest')
        else:
            data_in = data_input[:, self.start_dim: self.start_dim + self.dim, ...]
            if self.affine:
                data_in = torch.cat(
                    (data_in, torch.ones(data_input.shape[0], 1, *data_input.shape[2:], device=data_input.device,
                                         dtype=data_input.dtype)), 1
                )
            old_shape = data_in.shape
            data_in = data_in.view((*old_shape[:2], -1))
            result = torch.bmm(trans, data_in)
            if self.affine:
                result = result[:, :-1, ...] / result[:, -1, ...]
            result = result.view((old_shape[0], self.dim, *old_shape[2:]))
            result = torch.cat(
                (data_input[:, : self.start_dim, ...], result, data_input[:, self.start_dim + self.dim:, ...]), 1)
        return result


class L2ReflectHead(nn.Module):
    def __init__(self, in_channels, mid_channels, *args, **kwargs):
        super().__init__()
        self.net = nn.Sequential(
            md.DeFire(in_channels, mid_channels // 16, mid_channels // 2),
            md.Fire(mid_channels, mid_channels // 16, 16),
            md.Conv(32, 1, 1, relu=False, norm=False),
            nn.Sigmoid(),
        )

    def forward(self, data_input, features):
        out = self.net(features)
        if out.shape[-1] != data_input.shape[-1]:
            diff = out.shape[-1] - data_input.shape[-1]
            out = out[..., (diff // 2): -(diff // 2)]
        return (out,)


class ReflectHead(nn.Module):
    def __init__(self, in_channels, mid_channels, return_value=False):
        super().__init__()
        self.ranges = nn.Parameter(
            torch.tensor(
                [0.01, 0.04, 0.04, 0.04, 0.04, 0.08, 0.08, 0.08,
                 0.09, 0.5]),
            requires_grad=False,
        )
        self.mins = nn.Parameter(
            torch.tensor(
                [0.0, 0.01, 0.05, 0.09, 0.13, 0.17, 0.25, 0.33, 0.41,
                 0.5]),
            requires_grad=False,
        )
        self.up = md.DeFire(in_channels, mid_channels // 16, mid_channels // 2)
        self.clazz = md.Conv(mid_channels, 10, 1, relu=False, norm=False)
        self.sq = nn.Sequential(nn.ReLU(inplace=True), nn.BatchNorm2d(10), md.Fire(10, 2, 8))
        self.sm = md.Fire(mid_channels, mid_channels // 16, 8)
        self.dist = nn.Sequential(md.Conv(32, 1, 1, relu=False, norm=False), nn.Sigmoid())
        self.return_value = return_value

    def forward(self, data_input, features):
        up = self.up(features)
        if up.shape[-1] != data_input.shape[-1]:
            diff = up.shape[-1] - data_input.shape[-1]
            up = up[..., (diff // 2): -(diff // 2)]
        clazz = self.clazz(up)
        sq = self.sq(clazz)
        sm = self.sm(up)
        dist = self.dist(torch.cat((sq, sm), 1))
        if self.return_value:
            pred_bin = torch.argmax(clazz, 1, keepdim=True)
            torch.clamp_(pred_bin, 0, 9)
            value = self.mins[pred_bin] + dist * self.ranges[pred_bin]
            return clazz, dist, value
        return clazz, dist


class SegmentHead(nn.Module):
    def __init__(self, in_channels, mid_channels, num_classes, crf_iters, crf_start_dim, crf_dims, **crf_kwargs):
        super().__init__()
        self.net = nn.Sequential(
            md.DeFire(in_channels, mid_channels // 16, mid_channels // 2),
            md.Fire(mid_channels, mid_channels // 16, mid_channels // 2),
            md.Conv(mid_channels, num_classes, 1, relu=False, norm=False),
        )
        self.crf = md.CRF(crf_iters, crf_start_dim, crf_dims, **crf_kwargs)

    def forward(self, data_input, features):
        result = self.net(features)
        if result.shape[-1] != data_input.shape[-1]:
            diff = result.shape[-1] - data_input.shape[-1]
            result = result[..., (diff // 2): -(diff // 2)]
        result = self.crf(data_input, result)
        return result


def autopad(k, p=None, d=1):  # kernel, padding, dilation
    # Pad to 'same' shape outputs
    if d > 1:
        k = d * (k - 1) + 1 if isinstance(k, int) else [d * (x - 1) + 1 for x in k]  # actual kernel-size
    if p is None:
        p = k // 2 if isinstance(k, int) else [x // 2 for x in k]  # auto-pad
    return p


class WeatherClassifyHead(nn.Module):
    def __init__(self,
                 c1,
                 c2,
                 k=1,
                 s=1,
                 p=None,
                 g=1,
                 dropout_p=0.5):  # ch_in, ch_out, kernel, stride, padding, groups, dropout probability
        super().__init__()
        c_ = 1280  # efficientnet_b0 size
        self.conv = md.Conv(c1, c_, k, autopad(k, p), s, g)
        self.pool = nn.AdaptiveAvgPool2d(1)  # to x(b,c_,1,1)
        self.drop = nn.Dropout(p=dropout_p, inplace=True)
        self.linear = nn.Linear(c_, c2)  # to x(b,c2)

    def forward(self, x):
        if isinstance(x, list):
            x = torch.cat(x, 1)
        x = self.conv(x)
        x = self.pool(x)
        x = self.drop(x)
        x = self.linear(x.flatten(1))
        return x
