import random

import numpy as np
import psutil
import torch
from py3nvml import py3nvml

from . import _int_utils as iu


def seed_all(seed):
    random.seed(seed)
    torch.manual_seed(seed)
    np.random.seed(seed)


def get_available_memory(device, clear_before=False):
    if not isinstance(device, torch.device):
        device = torch.device(device)
    if device.type == 'cpu':
        return psutil.virtual_memory().available
    if clear_before:
        torch.cuda.empty_cache()
    index = device.index if device.index else 0
    mem = py3nvml.nvmlDeviceGetMemoryInfo(iu._NVML_MAP[index])
    torch_mem = torch.cuda.memory_cached(device) - torch.cuda.memory_allocated(device)
    return mem.free + torch_mem
