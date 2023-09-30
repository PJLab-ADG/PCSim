import warnings

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm

BBOX_CONNS = {
    '2D': frozenset([(0, 1), (0, 2), (1, 3), (2, 3)]),
    '3D': frozenset([(0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3), (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7)]),
}

_RGB2GS = np.array([[2126, 7152, 722]])


def rgb2gs(points, dim=0):
    if len(points.shape) > 2:
        target = len(points.shape) - 2
    else:
        target = 0
    points = np.moveaxis(points, dim, target)
    npoints = _RGB2GS @ points
    npoints /= 10000
    npoints = np.moveaxis(npoints, target, dim)
    return npoints


def tohomo(points):
    result = np.ones(tuple([points.shape[0] + 1] + list(points.shape[1:])), dtype=points.dtype)
    result[:-1, :] = points
    return result


def fromhomo(points, keep_all_pts=True, return_all_dims=False):
    result = np.copy(points)
    if not keep_all_pts:
        result = result[:, result[-1, :] > 0]
    result[:, result[-1] != 0] /= result[-1, result[-1] != 0]

    if return_all_dims:
        return result
    else:
        return result[:-1]


def rot_mat(world_rot, relative_rot=None, rads=False, dtype='f4'):
    if relative_rot is None:
        relative_rot = np.array([0, 0, 0])
    if not rads:
        angles = np.radians(np.array(world_rot) - np.array(relative_rot))
    else:
        angles = np.array(world_rot) - np.array(relative_rot)
    sines = np.sin(angles)
    coses = np.cos(angles)
    rot_x = np.array([[1, 0, 0], [0, coses[0], -sines[0]], [0, sines[0], coses[0]]])
    rot_y = np.array([[coses[1], 0, sines[1]], [0, 1, 0], [-sines[1], 0, coses[1]]])
    rot_z = np.array([[coses[2], -sines[2], 0], [sines[2], coses[2], 0], [0, 0, 1]])
    return (rot_z @ rot_y @ rot_x).astype(dtype)


def blend_img(background, overlay_rgba, gamma=2.2):
    alpha = overlay_rgba[:, :, 3]
    over_corr = np.float_power(overlay_rgba[:, :, :3], gamma)
    bg_corr = np.float_power(background, gamma)
    return (np.float_power(over_corr * alpha[..., None] + (1 - alpha)[..., None] * bg_corr, 1 / gamma)).astype(background.dtype)


def draw_bbox(draw, box, color, connections, width=2):
    color = tuple(color)
    for conn1, conn2 in connections:
        draw.line(box[conn1] + box[conn2], fill=color, width=width)


def cm_to_img(conf_mat, clazz=None, normalize=False, cmap=cm.Blues):
    if len(conf_mat.shape) != 2:
        raise ValueError('Confusion matrix needs to be 2 dimensional!')
    if conf_mat.shape[0] != conf_mat.shape[1]:
        warnings.warn('Confusion matrix should be square! Squaring it now')
        new_cm = np.zeros([max(conf_mat.shape)] * 2, dtype=conf_mat.dtype)
        new_cm[: conf_mat.shape[0], : conf_mat.shape[1]] = conf_mat
        conf_mat = new_cm
    if normalize:
        conf_mat = conf_mat / conf_mat.sum(axis=1)[:, np.newaxis]

    if clazz is None:
        clazz = np.arange(conf_mat.shape[0])

    fig, axes = plt.subplots()
    img = axes.imshow(conf_mat, interpolation='nearest', cmap=cmap)
    axes.figure.colorbar(img, ax=axes)
    axes.set(
        xticks=np.arange(conf_mat.shape[1]),
        yticks=np.arange(conf_mat.shape[0]),
        xticklabels=clazz,
        yticklabels=clazz,
        ylabel='True label',
        xlabel='Predicted label',
    )

    # Rotate the tick labels and set their alignment.
    plt.setp(axes.get_xticklabels(), rotation=45, ha='right', rotation_mode='anchor')

    # Loop over data dimensions and create text annotations.
    fmt = '.3f' if normalize else 'd'
    thresh = conf_mat.max() / 2.0
    for i in range(conf_mat.shape[0]):
        for j in range(conf_mat.shape[1]):
            axes.text(j, i, format(conf_mat[i, j], fmt), ha='center', va='center', color='white' if conf_mat[i, j] > thresh else 'black')
    fig.tight_layout()
    return fig
