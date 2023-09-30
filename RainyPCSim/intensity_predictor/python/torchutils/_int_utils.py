import functools
import operator
import os

import torch
from py3nvml import py3nvml

if torch.cuda.is_available():
    py3nvml.nvmlInit()

    def _get_nvml_cuda_mapping():
        vis = os.environ.get('CUDA_VISIBLE_DEVICES')
        if vis is None:
            return [py3nvml.nvmlDeviceGetHandleByIndex(i) for i in range(torch.cuda.device_count())]
        return [py3nvml.nvmlDeviceGetHandleByIndex(i) for i in map(int, vis.split(','))]

    _NVML_MAP = _get_nvml_cuda_mapping()


else:
    _NVML_MAP = []


def _product(iterable):
    return functools.reduce(operator.mul, iterable, 1)
