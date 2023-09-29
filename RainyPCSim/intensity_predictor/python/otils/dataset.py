import math
import os
import os.path as osp
import types
import warnings
import weakref

import attr

import otils.utils as ou

fw_orig = warnings.formatwarning  # pylint: disable=invalid-name
warnings.formatwarning = lambda msg, categ, fname, lineno, line=None: fw_orig(msg, categ, osp.split(fname)[1], lineno, '')


def _subclass_validator():
    # Ugly hiding because of circular validators
    def _valid(_, __, value):
        return issubclass(value, DatasetEntry)

    return _valid


class _mystr(str):
    '''Dummy class in order to be able to return str'''


@attr.s(kw_only=True)
class NumFiles:
    num_files = attr.ib(default=None, validator=attr.validators.optional(attr.validators.instance_of(int)))
    search_fun = attr.ib(default=None, validator=attr.validators.optional(attr.validators.is_callable()))

    def __attrs_post_init__(self):
        if bool(self.num_files is None) == bool(self.search_fun is None):
            raise ValueError('Exactly one of the fields can be specified')
        if self.search_fun is not None:
            self.num_files = self.search_fun()


@attr.s
class DataAttrib:
    '''
    Based off of property, allows to specify function for loading, recreating and on which id of the dataset to begin.
    '''

    _attrib_name = attr.ib(init=False)
    _own_name = attr.ib(init=False)
    _format = attr.ib(converter=ou.listify)
    _loader = attr.ib()
    _path_part = attr.ib(converter=ou.listify)
    _creator = attr.ib(default=None)
    _saver = attr.ib(default=None)
    _to_pass = attr.ib(default=attr.Factory(list))
    _start_id = attr.ib(default=0, converter=int, validator=attr.validators.instance_of(int))
    _stride = attr.ib(default=1, converter=int, validator=attr.validators.instance_of(int))
    _together = attr.ib(default=1, converter=int, validator=attr.validators.instance_of(int))
    _format_kwargs = attr.ib(default=attr.Factory(list))
    _wfable = attr.ib(default=True)
    _deletable = attr.ib(default=True)

    def __set_name__(self, owner, name):
        if not issubclass(owner, DatasetEntry):
            raise TypeError(f'You cannot set {self.__class__.__name__} to anything else but subclasses of {DatasetEntry.__name__}!')
        self._attrib_name = name  # Get the name of the attribute
        self._own_name = name
        setattr(owner, '_' + name + '_exists', _AttribEx(self))

    def _create_fname(self, inst):
        data_id = inst.data_id
        if self._stride is not None:
            offset = (data_id % self._stride) - self._start_id
        else:
            offset = inst.parent.num_files - 1
        if offset != 0:
            if self._together is None:
                real_data = getattr(inst.parent[self._start_id], self._attrib_name)
            elif offset < self._together:
                real_data = getattr(inst.parent[data_id - offset], self._attrib_name)
            else:
                real_data = None
            setattr(inst, '_' + self._attrib_name, real_data)
            return real_data
        format_kwargs = {kwname: getattr(inst, kwname, None) for kwname in self._format_kwargs}
        fname = osp.join(*ou.listify(inst.data_dir) + self._path_part + self._format)
        fname = _mystr(fname.format(data_id=data_id, width=inst.width, **format_kwargs))
        return fname

    def __get__(self, inst, owner=None):
        if inst is None:  # Get this object if not called on instance
            return self
        if not hasattr(inst, '_' + self._attrib_name):
            fname = self._create_fname(inst)  # Create name
            setattr(inst, '_' + self._attrib_name, fname)
        if isinstance(getattr(inst, '_' + self._attrib_name), _mystr):
            fname = getattr(inst, '_' + self._attrib_name)
            data = None
            try:
                data = self._loader(fname)  # Try to load it
            except OSError as exc:
                if self._creator is None:
                    warnings.warn(f'Could not load {fname} and cannot recreate it! Error was {exc}')
                    data = None
                else:
                    warnings.warn(f'Could not load {fname}, recreating it! Error was {exc}')
                    to_pass = dict()
                    for name in self._to_pass:
                        if not name.startswith('_'):
                            undername = '_' + name
                        else:
                            undername = name
                            name = name[1:]
                        if hasattr(self, undername):
                            to_pass[name] = getattr(self, undername)
                    data = self._creator(inst, **to_pass)  # Or recreate it
                    if self._saver is not None and getattr(inst, 'autosave', False):
                        os.makedirs(osp.dirname(fname), exist_ok=True)
                        self._saver(fname, data)  # And potentially save it
            setattr(inst, '_' + self._attrib_name, data)
        return getattr(inst, '_' + self._attrib_name)

    def __set__(self, inst, value):
        raise AttributeError(f'Can\'t set {self._own_name} in {inst.__class__.__name__}!')

    def __delete__(self, inst):
        if self._deletable:
            fname = self._create_fname(inst)
            os.remove(fname)
        else:
            raise AttributeError(f'Can\'t delete {self._own_name} from {inst.__class__.__name__}!')


class _AttribEx(DataAttrib):
    def __init__(self, parent):
        for name, value in attr.asdict(parent).items():
            if name == '_own_name':
                value = '_' + value + '_exists'
            setattr(self, name, value)

    def __set_name__(self, owner, name):
        if not issubclass(owner, DatasetEntry):
            raise TypeError(f'You cannot set {self.__class__.__name__} to anything else but subclasses of {DatasetEntry.__name__}!')
        self._own_name = name  # Get the name of the attribute

    def __get__(self, inst, owner=None):
        if inst is None:  # Get this object if not called on instance
            return self
        if not hasattr(inst, '_' + self._attrib_name):
            fname = self._create_fname(inst)
            return os.path.exists(fname)
        _name = getattr(inst, '_' + self._attrib_name)
        return not isinstance(_name, _mystr) or os.path.exists(_name)

    def __delete__(self, inst):
        raise AttributeError(f'Can\'t delete {self._own_name} from {inst.__class__.__name__}!')


@attr.s
class Dataset:
    '''
    Dataset class. Num_files can be either integer, or a function to compute number of files (i.e. search)
    '''

    base_dir = attr.ib()
    num_files = attr.ib(validator=attr.validators.instance_of(NumFiles))
    base_entry = attr.ib(validator=_subclass_validator())
    entry_args = attr.ib(default=attr.Factory(list))
    entry_kwargs = attr.ib(default=attr.Factory(dict))

    _data = attr.ib(init=False, default=attr.Factory(weakref.WeakValueDictionary))

    def __attrs_post_init__(self):
        self.num_files = self.num_files.num_files
        if 'width' not in self.entry_kwargs:
            self.entry_kwargs['width'] = math.ceil(math.log10(self.num_files + 1))
        for name, item in self.base_entry.__dict__.items():
            if isinstance(item, DataAttrib):
                setattr(self, name, DataAttribIter(self, item))

    def __len__(self):
        return self.num_files

    def __getitem__(self, key):
        if isinstance(key, slice):
            return [self[nkey] for nkey in range(*key.indices(self.num_files))]
        if isinstance(key, int):
            if key < 0:
                key += self.num_files
            if key < 0 or key >= self.num_files:
                raise IndexError(f'Key can be only in [{-self.num_files}, {self.num_files})')
            data = self._data.get(key, None)
            if data is None:
                data = self.base_entry(self, self.base_dir, key, *self.entry_args, **self.entry_kwargs)
                self._data[key] = data
            return data
        raise TypeError('Can index only by slice or int!')

    def __iter__(self):
        for i in range(len(self)):
            yield self[i]


@attr.s
class DatasetEntry:
    '''Base entry for one item within Dataset.
    You probably want to subclass it in order to add attributes for entries'''

    parent = attr.ib(validator=attr.validators.instance_of(Dataset))
    data_dir = attr.ib()
    data_id = attr.ib()
    width = attr.ib()
    autosave = attr.ib(default=True)


@attr.s
class DataAttribIter:
    '''
    Iterator over attributes. Should not be instantiated on its own, gets glued to Dataset
    '''

    parent = attr.ib(validator=attr.validators.instance_of(Dataset))
    entry = attr.ib(validator=attr.validators.instance_of(DataAttrib))
    num_files = attr.ib(init=False, default=None)
    _data = attr.ib(init=False)
    _reload = attr.ib(init=False, default=False)

    @_data.default
    def data_factory(self):
        if self.entry._wfable:  # pylint: disable=protected-access
            return weakref.WeakValueDictionary()
        return dict()

    def __attrs_post_init__(self):
        self.num_files = (self.parent.num_files - self.entry._start_id) // self.entry._stride  # pylint: disable=protected-access
        if 'exists' in self.entry._own_name:  # pylint: disable=protected-access
            self._reload = True

    def _translate_key(self, key):
        return key * self.entry._stride + self.entry._start_id  # pylint: disable=protected-access

    def __len__(self):
        return self.num_files

    def __getitem__(self, key):
        if isinstance(key, slice):
            return [self[nkey] for nkey in range(*key.indices(self.num_files))]
        if isinstance(key, int):
            if key < 0:
                key += self.num_files
            if key < 0 or key >= self.num_files:
                raise IndexError(f'Key can be only in [{-self.num_files}, {self.num_files})')
            key = self._translate_key(key)
            if self._reload:
                data = getattr(self.parent[key], self.entry._own_name)  # pylint: disable=protected-access
            else:
                data = self._data.get(key, None)
                if data is None:
                    data = getattr(self.parent[key], self.entry._own_name)  # pylint: disable=protected-access
                    self._data[key] = data
            return data
        raise TypeError('Can index only by slice or int!')

    def __iter__(self):
        for i in range(len(self)):
            yield self[i]


__all__ = [name for name in globals() if not (name.startswith('_') or isinstance(globals()[name], types.ModuleType))]
