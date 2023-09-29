import sys as _sys
import types as _types

import otils as _ou

from ._data import *  # noqa: F403,F401
from ._modules import *  # noqa: F403,F401
from ._registry import *  # noqa: F403,F401
from ._utils import *  # noqa: F403,F401

__all__ = [name for name in globals() if not (name.startswith('_') or isinstance(globals()[name], _types.ModuleType))]

_ou.utils.delete_all(
    _sys.modules[__name__],
    [
        lambda name, obj: isinstance(obj, _types.ModuleType),
        lambda name, obj: hasattr(obj, '__module__') and not obj.__module__.startswith('torchutils'),
    ],
    first_run_exclude=['_types', '_sys', '_ou'],
)
