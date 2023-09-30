import typing as _t
from functools import partial

import torch
import torch.nn as nn

_registry_base = {
    'Activ': {
        'relu': nn.ReLU,
        'selu': nn.SELU,
        'rrelu': nn.RReLU,
        'prelu': nn.PReLU,
        'lrelu': nn.LeakyReLU,
        'sigmoid': nn.Sigmoid,
        'elu': nn.ELU,
        'tanh': nn.Tanh,
        'softmax': nn.Softmax,
        'logsoftmax': nn.LogSoftmax,
    },
    'Norm1d': {'bn': nn.BatchNorm1d, 'in': nn.InstanceNorm1d},
    'Norm2d': {'bn': nn.BatchNorm2d, 'in': nn.InstanceNorm2d},
    'EdgeAgg': {'sum': partial(torch.sum, dim=-1), 'mean': partial(torch.mean, dim=-1), 'max': lambda x: torch.max(x, dim=-1)[0]},
}


class _ModuleGroup(nn.Module):
    def __init__(self, name, *args, **kwargs):
        super().__init__()
        m = self.__class__.MODULES[name]
        if isinstance(m, type):
            self.module = m(*args, **kwargs)
            self.stored_args = None
            self.stored_kwargs = None
        else:
            self.module = m
            self.stored_args, self.stored_kwargs = args, kwargs

    def forward(self, *args, **kwargs):
        if self.stored_args is not None:
            return self.module(*args, *self.stored_args, **kwargs, **self.stored_kwargs)
        return self.module(*args, **kwargs)


class _Registry:
    def __init__(self, mapping):
        self.__groups = mapping
        self.__registry = dict()
        self.__rebuild_registry()

    def __getattr__(self, name: str) -> _t.Type[_ModuleGroup]:
        return self.__registry[name]

    def __rebuild_registry(self):
        for g in self.__groups:
            self.__registry[g] = type(g, (_ModuleGroup,), {'MODULES': self.__groups[g]})

    def __add_impl(self, group, name, fn):
        if group not in self.__groups:
            self.__groups[group] = dict()
        self.__groups[group][name] = fn
        self.__rebuild_registry()

    def add_to_registry(self, group, name, fn=None):
        if fn is None:

            def add_fn(fn):
                self.__add_impl(group, name, fn)
                return fn

            return add_fn
        self.__add_impl(group, name, fn)


registry = _Registry(_registry_base)
