import os.path as _osp
import warnings as _w

from . import checkpoint, dataset, io, utils, visual  # noqa : F401

_fw_orig = _w.formatwarning
_w.formatwarning = lambda msg, categ, fname, lineno, line=None: _fw_orig(msg, categ, _osp.split(fname)[1], lineno, '')

__all__ = [name for name in globals() if not name.startswith('_')]
