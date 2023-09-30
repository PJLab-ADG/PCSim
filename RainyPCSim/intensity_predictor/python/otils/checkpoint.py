import atexit
import datetime
import importlib
import inspect
import io
import json
import os.path as osp
import shutil
import sys
import tarfile as tf
import tempfile
import types
import typing
import warnings

import dill  # nosec

import otils.utils as ou

_META_TAR_NAME = 'names.json'
_DATA_TAR_NAME = 'data.pkl'


def _add_bytes_to_tarfile(tarf, data, arcname):
    bfile = io.BytesIO(data)
    tinfo = tf.TarInfo(arcname)
    tinfo.size = len(data)
    tarf.addfile(tinfo, bfile)


def _tarinfo_filter(tarinfo: tf.TarInfo) -> typing.Optional[tf.TarInfo]:
    if '__pycache__' in tarinfo.name:
        return None
    return tarinfo


def store_checkpoint(
    basename, data=None, modules: typing.Optional[typing.Sequence[types.ModuleType]] = None, time_format='%y-%m-%d--%H-%M-%S', overwrite=False
):
    if overwrite:
        mode = 'w'
    else:
        mode = 'x'
    if modules is None:
        modules = []
    if time_format is not None:
        tarname = f'{basename}-{datetime.datetime.now():{time_format}}.tar'
    else:
        tarname = f'{basename}.tar'
    names: typing.Set[str] = set()
    with tf.open(tarname, mode, dereference=True) as tarf:
        for mod in modules:
            modpackage = mod.__package__
            modfilename = mod.__file__
            modname = mod.__name__
            added_name = modpackage if modpackage else modname
            if added_name in names:
                continue
            if modname == osp.split(modfilename)[1].split('.')[0]:
                tarf.add(modfilename, osp.split(modfilename)[1])
            else:
                tarf.add(osp.split(modfilename)[0], modpackage, filter=_tarinfo_filter)
            names.add(added_name)
        _add_bytes_to_tarfile(tarf, json.dumps(ou.listify(names)).encode('utf-8'), _META_TAR_NAME)
        _add_bytes_to_tarfile(tarf, dill.dumps(data), _DATA_TAR_NAME)


def load_checkpoint(filename, reload_modules=True):
    dirname = tempfile.mkdtemp()
    sys.path.insert(0, dirname)
    atexit.register(shutil.rmtree, dirname, ignore_errors=True)  # Make sure to delete loaded modules
    stack = [x.frame for x in inspect.stack()[1:]]
    with tf.open(filename, 'r') as tarf:
        if reload_modules:
            meta = json.load(tarf.extractfile(_META_TAR_NAME))
            for name in list(sys.modules.keys()):  # Delete all names from sys.modules
                if name.split('.')[0] in meta:
                    del sys.modules[name]
            for member in tarf.getmembers():  # Extract all files from archive
                if member.name in (_META_TAR_NAME, _DATA_TAR_NAME):
                    continue
                tarf.extract(member, path=dirname)
            for name in meta:
                try:
                    module = importlib.import_module(name)  # Import module
                    for frame in stack:  # And for every frame
                        for key, mod in frame.f_globals.items():
                            if not isinstance(mod, types.ModuleType):
                                continue
                            if mod.__name__ == module.__name__:  # Find if the module with the same name is already there, and if it is, reload
                                frame.f_globals[key] = module
                                break
                except ImportError:
                    warnings.warn(f'Failed to reload module {name}')
        data = dill.load(tarf.extractfile(_DATA_TAR_NAME))  # nosec # get data
        del sys.path[0]
    return data


__all__ = [name for name in globals() if not (name.startswith('_') or isinstance(globals()[name], types.ModuleType))]
