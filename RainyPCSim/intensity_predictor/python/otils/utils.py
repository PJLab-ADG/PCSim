import itertools as it
import random
import string
import types
import typing


class Singleton(type):
    _instances: typing.MutableMapping[type, 'Singleton'] = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


def listify(data: typing.Union[typing.Iterable, str]) -> typing.List[str]:
    if data is None:
        return ['']
    if isinstance(data, typing.Iterable):
        if not isinstance(data, str):
            return list(data)
    return [data]


def chunk(data: typing.Iterable, chunk_size: int = 1) -> typing.Iterable[typing.Iterable]:
    i = 0
    result = []
    for x in data:
        result.append(x)
        i += 1
        if i is chunk_size:
            yield result
            i = 0
            result = []
    if result:
        yield result


def flatten(iterable):
    return it.chain.from_iterable(iterable)


def rand_string(hash_len=10, choice=string.ascii_lowercase + string.digits):
    return ''.join(random.choices(choice, k=hash_len))


def delete_all(where, conditions=None, exclude=None, first_run_exclude=None):
    if first_run_exclude is None:
        first_run_exclude = set()
    else:
        first_run_exclude = set(first_run_exclude)
    if conditions is None:
        conditions = []
    if exclude is None:
        exclude = []
    for name in set(dir(where)) - first_run_exclude:
        if name in exclude:
            continue
        if len(name) >= 2 and name[0] == '_' and name[1] != '_':
            delattr(where, name)
            continue
        for cond in conditions:
            if cond(name, getattr(where, name)):
                delattr(where, name)
                break
    for name in first_run_exclude:
        if hasattr(where, name):
            delattr(where, name)


__all__ = [name for name in globals() if not (name.startswith('_') or isinstance(globals()[name], types.ModuleType))]
