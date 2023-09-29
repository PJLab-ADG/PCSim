import argparse
import builtins
import collections
import contextlib
import enum
import functools
import os.path as osp
import random
import weakref

import numpy as np
import torch
import torch.optim
import torch.utils.data as data


try:
    import tqdm

    _TQDM_FOUND = True
except ImportError:
    _TQDM_FOUND = False

builtins.print = functools.partial(print, flush=True)


def _compose(*functions):
    return functools.reduce(lambda f, g: lambda x: f(g(x)), functions, lambda x: x)


class SimpleDataset(data.Dataset):
    class weakdict(dict):
        __slots__ = ('__weakref__',)

    class weaklist(list):
        __slots__ = ('__weakref__',)

    def __init__(self, folder: str, name=None, ext='.npy', shuffle=True, keep_ram=False):
        self.folder = folder
        self.ext = ext.lower()
        if name is None:
            name = osp.split(folder)[1]
        self.name = name
        self.files = self.scan_files()
        if shuffle:
            random.shuffle(self.files)
        self._len = len(self.files)
        self.keep_ram = keep_ram
        if keep_ram:
            self.loaded = dict()
        else:
            self.loaded = weakref.WeakValueDictionary()

    def __len__(self):
        return self._len

    def __getitem__(self, key):
        key = int(key)
        fname = self.files[key]
        data_item = self.loaded.get(fname, None)
        if data_item is None:
            with self.__weakcm():
                data_item = self.load_and_transform(fname, key)
            self.loaded[fname] = data_item
        return data_item

    def load_and_transform(self, fname, key):
        raise NotImplementedError

    def scan_files(self):
        raise NotImplementedError

    @contextlib.contextmanager
    def __weakcm(self):
        if not self.keep_ram:
            _stash = dict, list
            builtins.dict = self.weakdict
            builtins.list = self.weaklist
            yield
            builtins.dict, builtins.list = _stash
        else:
            yield


class TorchMode(enum.Enum):
    TRAIN = enum.auto()
    EVAL = enum.auto()


def dict_to_cuda(d, *args, **kwargs):
    for key in d:
        if hasattr(d[key], 'cuda'):
            d[key] = d[key].cuda(*args, **kwargs)
    return d


class Runner:
    def __init__(
            self,
            model,
            loss_fn,
            optimizer,
            pass_keys,
            gt_keys,
            embedder=None,
            embed_channels=None,
            verbose=False,
            args=None,
            use_tqdm=False,
            accum_losses=False,
            pass_as_kwargs=False,
            cat_channels=False,
    ):
        self.model = model
        self.loss_fn = loss_fn
        self.optimizer = optimizer
        self.scheduler = None
        self.pass_keys = pass_keys
        self.gt_keys = gt_keys
        self.verbose = verbose
        self.use_tqdm = use_tqdm & _TQDM_FOUND
        self.embedder = embedder
        self.embed_channels = embed_channels
        if args is None:
            args = argparse.Namespace(cuda=False, keep_ram=False)
        self.args = args
        try:
            self.cuda = args.cuda
        except AttributeError:
            self.cuda = False
        try:
            self.keep_ram = args.keep_ram
        except AttributeError:
            self.keep_ram = False
        if self.use_tqdm:
            self.iter_wrap = tqdm.tqdm
        else:
            self.iter_wrap = functools.partial(map, lambda x, *args, **kwargs: x)
        self.pass_as_kwargs = pass_as_kwargs
        self.accum_losses = accum_losses
        self.run_times = collections.Counter()
        self.run_losses = dict()
        self.cat_channels = cat_channels
        self.writer = None

    @contextlib.contextmanager
    def _setup(self):
        _stash = builtins.print
        if self.use_tqdm:
            builtins.print = _compose(lambda x: tqdm.tqdm.write(x) if x.strip() else None,
                                      lambda x: '' if not self.verbose else x, str)
        else:
            builtins.print = _compose(lambda x: _stash(x) if x.strip() else None,
                                      lambda x: '' if not self.verbose else x, str)
        yield
        builtins.print = _stash

    def __call__(self, dataloader, mode):
        datalen = len(dataloader.dataset)
        did = 0
        self.model = self.model.train() if mode is TorchMode.TRAIN else self.model.eval()
        self.run_pre_epoch(dataloader.dataset, mode)
        if self.accum_losses:
            self.run_losses[dataloader.dataset] = list()
        with self._setup():
            with torch.set_grad_enabled(mode == TorchMode.TRAIN):
                for batch_id, batch in self.iter_wrap(enumerate(dataloader), total=len(dataloader)):
                    batch_len = len(next(iter(batch.values())))
                    did += batch_len
                    if self.cuda:
                        dict_to_cuda(batch, **({'non_blocking': True} if self.keep_ram else {}))
                    if mode == TorchMode.TRAIN:
                        self.optimizer.zero_grad()
                    if self.embedder is not None:
                        for idx, channel in enumerate(self.embed_channels):
                            batch[channel + '_embed'] = self.embedder[idx](batch[channel])
                    run_kwargs = collections.OrderedDict((key, batch[key]) for key in self.pass_keys)
                    if self.cat_channels:
                        output = self.model(torch.cat(tuple(run_kwargs.values()), 1))
                    elif self.pass_as_kwargs:
                        output = self.model(**run_kwargs)
                    else:
                        output = self.model(*run_kwargs.values())
                    if mode == TorchMode.TRAIN or all([key in batch for key in self.gt_keys]):
                        loss_kwargs = collections.OrderedDict((key, batch[key]) for key in self.gt_keys)
                        if self.pass_as_kwargs:
                            loss, loss_dict = self.loss_fn(output, **loss_kwargs)
                        else:
                            loss, loss_dict = self.loss_fn(output, *loss_kwargs.values())
                        if mode == TorchMode.TRAIN:
                            loss.backward()
                            self.optimizer.step()
                    else:
                        loss = None
                        loss_dict = None
                    print_str = f'Epoch id: {self.run_times[dataloader.dataset]}\t{did: 6d} / {datalen : 6d}\t'
                    if loss is not None:
                        print_str += f'Loss: {loss:8.04f}\t'
                    if self.accum_losses:
                        self.run_losses[dataloader.dataset].append(loss.detach().cpu().numpy() * batch_len)
                        total_loss = np.sum(self.run_losses[dataloader.dataset])
                        print_str += f'Mean loss over epoch: {total_loss / did: 8.04f}\t'
                    extra = self.run_after_iter(batch, output, loss_dict, mode, did, batch_id, len(dataloader),
                                                dataloader.dataset)
                    if extra is not None:
                        print_str += str(extra)
                    print(print_str)
                print_str = f'Epoch {self.run_times[dataloader.dataset]} for dataset {dataloader.dataset} and mode {mode} finished!\n'
                extra = self.run_after_epoch(dataloader.dataset, mode)
                if extra is not None:
                    print_str += str(extra)
                print(print_str)
                self.run_times[dataloader.dataset] += 1

    def run_after_iter(self, batch, output, loss, mode, did, batch_id, total_batches, dataset):
        pass

    def run_after_epoch(self, dataset, mode):
        pass

    def run_pre_epoch(self, dataset, mode):
        pass
