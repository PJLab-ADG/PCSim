import json
import os.path as osp
import types

import hiyapyco
import numpy as np
import yaml
from PIL import Image


def img_load(filename, load_dtype='<f4', return_dtype='<f4', div=255.0):
    return (np.array(Image.open(filename), dtype=load_dtype) / div).astype(return_dtype)


def img_save(filename, img):
    Image.fromarray(img).save(filename)


def read_json(filename):
    with open(filename, 'rt', encoding='utf-8') as f:
        return json.load(f)


def np_savez(filename, arrs):
    np.savez_compressed(filename, **arrs)


def np_load(filename):
    arr = np.load(filename)
    if isinstance(arr, np.lib.npyio.NpzFile):
        return dict(arr)
    return arr


def _include(loader, node):
    filename = osp.join(osp.dirname(loader.stream.name), loader.construct_scalar(node))
    with open(filename, 'r') as f:
        return yaml.safe_load(f)


hiyapyco.odyldo.ODYL.add_constructor('!include', _include)


def load_multi_yml(filename, merge=True):
    basename = osp.dirname(filename)
    with open(filename, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    files = [osp.join(basename, d) for d in data]
    return hiyapyco.load(*files, method=hiyapyco.METHOD_MERGE if merge else hiyapyco.METHOD_SIMPLE, mergelists=False, usedefaultyamlloader=False)


__all__ = [name for name in globals() if not (name.startswith('_') or isinstance(globals()[name], types.ModuleType))]
