import typing as _t
from collections import abc

import torch
import torch.nn as nn
import torch.nn.functional as F

from ._registry import registry


class ModuleBase(nn.Module):
    def _apply_patch(_fn_name):  # pylint: disable=no-self-argument  # monkey patching dark-magic
        def fn(self, *args, **kwargs):
            result = getattr(super(), _fn_name)(*args, **kwargs)
            for c in self.children():
                getattr(c, _fn_name)(*args, **kwargs)
            return result

        return fn

    to = _apply_patch('to')
    cpu = _apply_patch('cpu')
    cuda = _apply_patch('cuda')


class Sequential(ModuleBase, nn.Sequential):
    def forward(self, data):
        for module in self._modules.values():
            if isinstance(data, _t.Sequence):
                data = module(*data)
            else:
                data = module(data)
        return data


@registry.add_to_registry('Activ', 'none')
@registry.add_to_registry('Norm1d', 'none')
@registry.add_to_registry('Norm2d', 'none')
class Identity(ModuleBase):
    def __init__(self, *args, **kwargs):
        super().__init__()

    def forward(self, data):
        return data


class Functional(ModuleBase):
    def __init__(self, function, *args, **kwargs):
        super().__init__()
        self.args = args
        self.kwargs = kwargs
        self.function = function

    def forward(self, x):
        return self.function(x, *self.args, **self.kwargs)

    def __repr__(self):
        return 'Functional(' + self.function.__name__ + ')'


class AddNoise(ModuleBase):
    def __init__(self, channels, init=True):
        super().__init__()
        self.weights = nn.Parameter(torch.empty(1, channels, 1, 1))
        if init:
            nn.init.normal_(self.weights)

    def forward(self, x):
        return x + self.weights * torch.rand(x.shape, device=x.device, requires_grad=False)


class ConvBlock(ModuleBase):
    def __init__(
        self,
        in_channels: int,
        out_channels: int,
        kernel: _t.Union[int, _t.Sequence[int]],
        activ: str,
        norm: str,
        dim: int = 1,
        noise: bool = False,
        pad_mode: str = 'default',
        bias=True,
        conv_kwargs: _t.Optional[_t.MutableMapping] = None,
        activ_kwargs: _t.Optional[_t.Mapping] = None,
        norm_kwargs: _t.Optional[_t.Mapping] = None,
    ):
        super().__init__()
        if activ_kwargs is None:
            activ_kwargs = {}
        if norm_kwargs is None:
            norm_kwargs = {}
        if conv_kwargs is None:
            conv_kwargs = {}

        modules: _t.MutableSequence[nn.Module] = []

        if 'padding' in conv_kwargs and pad_mode != 'default':
            padding = conv_kwargs.pop('padding')
            if not isinstance(padding, abc.Sequence):
                padding = [padding] * 4
            modules.append(Functional(F.pad, padding, mode=pad_mode))

        conv_mod = getattr(nn, f'Conv{dim}d')
        modules.append(conv_mod(in_channels, out_channels, kernel, bias=bias, **conv_kwargs))
        if noise:
            modules.append(AddNoise(out_channels))
        modules.append(registry.Activ(activ, **activ_kwargs))
        norm_class = getattr(registry, f'Norm{dim}d')
        modules.append(norm_class(norm, out_channels, **norm_kwargs))

        self.run = nn.Sequential(*modules)

    def forward(self, x):
        return self.run(x)


class LinBlock(ModuleBase):
    def __init__(
        self,
        in_channels: int,
        out_channels: int,
        activ: str,
        norm: str,
        noise: bool = False,
        bias: bool = True,
        activ_kwargs: _t.Optional[_t.Mapping] = None,
        norm_kwargs: _t.Optional[_t.Mapping] = None,
    ):
        super().__init__()
        if activ_kwargs is None:
            activ_kwargs = {}
        if norm_kwargs is None:
            norm_kwargs = {}

        modules: _t.MutableSequence[nn.Module] = []

        modules.append(nn.Linear(in_channels, out_channels, bias))
        if noise:
            modules.append(AddNoise(out_channels))
        modules.append(registry.Activ(activ, **activ_kwargs))
        if norm != 'in':
            modules.append(registry.Norm1d(norm, out_channels, **norm_kwargs))

        self.run = nn.Sequential(*modules)

    def forward(self, x):
        return self.run(x)
