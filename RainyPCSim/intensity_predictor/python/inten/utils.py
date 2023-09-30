import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F


def create_optim_from_kwargs(params, name='', **kwargs):
    optim = getattr(torch.optim, name, None)
    if optim is None:
        raise RuntimeError(f'Optimizer {name} was not found!')
    return optim(params, **kwargs)  # pylint: disable=not-callable


def create_loss_from_kwargs(reflect=False, gamma=2, l2_weight=0.5, ignore_index=4, only_l2=False, weather=False):
    if reflect:
        if only_l2:

            def fn(output, mask=None, labels=None, mean=True, **kwargs):
                intensity = kwargs['intensity']
                rgb_mask = kwargs.get('rgb_mask', None)
                if rgb_mask is None:
                    rgb_mask = torch.ones_like(intensity)
                if mask is None:
                    mask = torch.ones_like(intensity)
                rgb_mask = rgb_mask > 0
                mask = mask > 0
                if labels is not None:
                    label_mask = ~(labels == ignore_index)[:, None, ...]
                else:
                    label_mask = torch.ones_like(mask)
                full_mask = rgb_mask & mask & label_mask
                *_, pred = output
                loss = F.mse_loss(pred, intensity, reduction='none')[full_mask]
                if mean:
                    loss = torch.mean(loss)
                    return loss, {'total_loss':loss}
                return loss, {'total_loss':loss}

        else:
            def fn(output, mask=None, labels=None, mean=True, **kwargs):
                intensity_bin = kwargs['intensity_bin']
                intensity_dist = kwargs['intensity_dist']
                rgb_mask = kwargs.get('rgb_mask', None)
                if rgb_mask is None:
                    rgb_mask = torch.ones_like(intensity_dist)
                if mask is None:
                    mask = torch.ones_like(intensity_dist)
                rgb_mask = rgb_mask > 0
                mask = mask > 0
                if labels is not None:
                    label_mask = ~(labels == ignore_index)[:, None, ...]
                else:
                    label_mask = torch.ones_like(mask)
                if weather:
                    pred_ref = output[0]
                    pred_weather = output[-1]
                else:
                    pred_ref = output
                    pred_weather = None
                pred_bin, pred_dist, *_ = pred_ref
                pred_prob = F.softmax(pred_bin, 1)
                ce_loss = F.cross_entropy(pred_bin, intensity_bin, reduction='none')[:, None, ...]
                weight = (1 - pred_prob.gather(1, intensity_bin[:, None, ...])) ** gamma
                l2_loss = F.mse_loss(pred_dist, intensity_dist, reduction='none')
                # mm = torch.logical_and(torch.logical_and(rgb_mask,mask),label_mask)
                # print((rgb_mask & mask & label_mask).detach().cpu().numpy())
                relf_loss = weight * ce_loss + l2_loss * l2_weight
                valid_mask = rgb_mask & mask & label_mask
                loss = relf_loss[valid_mask]
                l2_loss = l2_loss[valid_mask]
                ce_loss = ce_loss[valid_mask]
                if weather:
                    weather_gt = kwargs['weather']
                    weather_gt = weather_gt[:, 0, 0]
                    weather_ce_loss = F.cross_entropy(pred_weather, weather_gt, reduction='none')[:, None, ...]
                    weather_ce_loss = weather_ce_loss.unsqueeze(2).unsqueeze(3).repeat([1] + list(mask.shape)[1:])
                    if mean:
                        loss, weather_ce_loss = torch.mean(loss), torch.mean(weather_ce_loss)
                        l2_loss,ce_loss = torch.mean(l2_loss), torch.mean(ce_loss)
                    total_loss = loss * 0.95 + weather_ce_loss * 0.05
                    return total_loss, {'total_loss':total_loss, 'reflect_loss': loss, 'weather_loss':weather_ce_loss, 'reflect_l2': l2_loss, 'reflect_ce':ce_loss}
                else:
                    if mean:
                        loss = torch.mean(loss)
                        l2_loss,ce_loss = torch.mean(l2_loss), torch.mean(ce_loss)
                    return loss, {'total_loss':loss, 'reflect_loss': loss, 'reflect_l2': l2_loss, 'reflect_ce':ce_loss}

    else:
        weight = np.array([0.132528185, 808.046146, 3.53494246, 188.286384], dtype='f4')

        def fn(output, mask=None, labels=None, mean=True, **kwargs):  # pylint: disable=unused-argument
            if mask is None:
                mask = torch.ones_like(output)
            mask = mask > 0
            tweight = torch.from_numpy(weight).to(output.device)
            b, _, h, w = output.shape
            output_prob = F.softmax(output, 1)
            output_prob = torch.cat(
                (output_prob, torch.ones(b, 1, h, w, device=output_prob.device, dtype=output_prob.dtype)), 1)
            result = F.cross_entropy(output, labels, reduction='none', weight=tweight, ignore_index=ignore_index)[:,
                     None, ...]
            loss_weight = (1 - output_prob.gather(1, labels[:, None, ...])) ** gamma
            loss = (loss_weight * result)[mask]
            if mean:
                return torch.mean(loss)
            return loss

    return fn


def create_image_fn(reflect, ignore_index=4):
    def renumpy(tensor, i, trans=True):
        vec = tensor[i].cpu().numpy()
        if trans:
            vec = np.transpose(vec, (1, 2, 0))
        return vec

    COLORS = np.array([(0, 0, 0), (255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255)], dtype='u1')
    NONE = np.array((128, 128, 128), dtype='u1')
    OK_COLORS = np.array([(0, 255, 0), (255, 0, 0)], dtype='u1')
    BORDER = 5
    if reflect:

        def fn(batch, output, i):
            *_, pred_value = output
            mask = np.squeeze(renumpy(batch['mask'], i) > 0)
            try:
                rgb_mask = np.squeeze(renumpy(batch['rgb_mask'], i) > 0)
            except KeyError:
                rgb_mask = np.ones_like(mask)
            try:
                label_mask = ~(renumpy(batch['labels'], i, False) == ignore_index)
            except KeyError:
                label_mask = np.ones_like(mask)
            mask = mask & rgb_mask & label_mask
            mask = np.squeeze(renumpy(batch['mask'], i) > 0)
            pred_value = np.squeeze(renumpy(pred_value, i))
            intensity = np.squeeze(renumpy(batch['intensity'], i))
            pred_value[~mask] = -1
            intensity[~mask] = -1
            ok_map = np.abs(pred_value - intensity)
            score = (ok_map[mask] ** 2).mean()
            h, w, *_ = intensity.shape
            result = np.ones((h * 3 + 2 * BORDER, w))
            result[:h] = intensity
            result[h + BORDER: 2 * h + BORDER] = pred_value
            result[-h:] = ok_map
            ok_map[~mask] = -1
            xyz = renumpy(batch['xyz'], i) * (131 * 2) - 131
            diff_pc = np.concatenate([xyz, ok_map[...,None]], axis=-1)
            src_pc = np.concatenate([xyz, intensity[...,None]], axis=-1)
            pred_pc = np.concatenate([xyz, pred_value[...,None]], axis=-1)
            return (result * 255).astype('u1'), (src_pc, pred_pc, diff_pc), score

    else:

        def fn(batch, output, i):
            output = torch.argmax(output, 1, keepdim=False)
            output = renumpy(output, i, False)
            labels = renumpy(batch['labels'], i, False)
            mask = np.squeeze(renumpy(batch['mask'], i) > 0) & ~(labels == ignore_index)
            pred = COLORS[output]
            pred[~mask] = NONE
            ok_map = ~(output == labels)
            score = ok_map[mask].mean()
            ok_map = OK_COLORS[ok_map.astype('u1')]
            ok_map[~mask] = NONE
            correct = COLORS[labels]
            correct[~mask] = NONE
            h, w, c = ok_map.shape
            result = np.ones((h * 3 + 2 * BORDER, w, c), dtype='u1') * 255
            result[:h] = correct
            result[h + BORDER: 2 * h + BORDER] = pred
            result[-h:] = ok_map
            return result, score

    return fn


def info_fn(reflect, ignore_index=4, num_classes=4):
    if reflect:

        def fn(batch, output):
            pred_value = output[-1].detach()
            mask = batch['mask'].detach() > 0
            if 'rgb_mask' in batch:
                rgb_mask = batch['rgb_mask'].detach() > 0
            else:
                rgb_mask = torch.ones_like(mask)
            if 'labels' in batch:
                label_mask = ~(batch['labels'].detach() == ignore_index)[:, None, ...]
            else:
                label_mask = torch.ones_like(mask)
            mask = mask & rgb_mask & label_mask
            intensity = batch['intensity'].detach()
            diff = (intensity - pred_value) * (intensity - pred_value)
            diff[~mask] = 0
            return torch.tensor([diff.sum(), mask.sum()])
    else:
        def fn(batch, output):
            output = torch.argmax(output.detach(), 1, keepdim=False)
            labels = batch['labels'].detach()
            mask = torch.squeeze(batch['mask'].detach() > 0, 1)
            label_mask = ~(labels == ignore_index)
            full_mask = mask & label_mask
            acc = (~(output == labels))[full_mask]
            stats = []
            for i in range(num_classes):
                tp = (output[full_mask & (labels == i)] == i).sum()
                fp = (labels[full_mask & (output == i)] != i).sum()
                fn = (output[full_mask & (labels == i)] != i).sum()
                stats.extend((tp, fp, fn))
            return torch.tensor([acc.sum(), *stats, full_mask.sum()])

    return fn


def scheduler(config, optimizer):
    name = config['name']
    sched = getattr(torch.optim.lr_scheduler, name, None)
    if sched is None:
        return None
    del config['name']
    if 'eta_min' in config:
        config['eta_min'] = float(config['eta_min'])
    return sched(optimizer, **config)


class Embed(nn.Embedding):
    def __init__(self, config):
        super().__init__(**config)

    def forward(self, data):
        result = super().forward(data)
        return result.permute(0, 3, 1, 2)
