import torch.nn as nn
import torch.nn.functional as F
import yaml

from . import heads
from . import modules as md
from .heads import WeatherClassifyHead


class SqueezeWithHead(nn.Module):
    def __init__(self, head_cls, squeeze_kwargs, head_kwargs):
        super().__init__()
        self.squeeze = SqueezeSegBone(**squeeze_kwargs)
        self.head = head_cls(**head_kwargs)

    def forward(self, x):
        features = self.squeeze(x)
        return self.head(x, features)

    @classmethod
    def load_from_kwargs(cls, data):
        if isinstance(data['head_cls'], str):
            head_cls = getattr(heads, data['head_cls'], None)
            if head_cls is None:
                raise RuntimeError(f'Could not find your class {data["head_cls"]}!')
            data['head_cls'] = head_cls
        return cls(**data)

    @classmethod
    def load_from_yaml(cls, yaml_file):
        with open(yaml_file, 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)
        return cls.load_from_kwargs(data)


class SqueezeWithMutiHead(nn.Module):
    def __init__(self, head_cls, squeeze_kwargs, head_kwargs):
        super().__init__()
        self.squeeze = SqueezeSegBoneRaw(**squeeze_kwargs)
        self.head = head_cls(**head_kwargs)
        self.weather_head = WeatherClassifyHead(640, 4, dropout_p=0.2)

    def forward(self, x):
        mid, feat = self.squeeze(x)
        reflect = self.head(x, feat)
        weather_cls = self.weather_head(mid)
        return reflect, weather_cls

    @classmethod
    def load_from_kwargs(cls, data):
        if isinstance(data['head_cls'], str):
            head_cls = getattr(heads, data['head_cls'], None)
            if head_cls is None:
                raise RuntimeError(f'Could not find your class {data["head_cls"]}!')
            data['head_cls'] = head_cls
        return cls(**data)

    @classmethod
    def load_from_yaml(cls, yaml_file):
        with open(yaml_file, 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)
        return cls.load_from_kwargs(data)


class SqueezeSegBone(nn.Module):
    def __init__(self, input_channels, squeeze_depth=2, cam_depth=1, conv_starts=64, squeeze_start=16, ef_start=64):
        super().__init__()
        self.reduce = 1
        self.start = nn.Sequential(
            md.Conv(input_channels, conv_starts, 3, 1, 2, top_parent=self),
            md.ContextAggregation(conv_starts, top_parent=self),
            md.Conv(conv_starts, conv_starts, 1, top_parent=self),
        )
        self.rest = nn.Sequential(
            md.Pool(3, 2, 1, top_parent=self),
            md.SqueezePart(conv_starts, squeeze_start, ef_start, squeeze_depth, cam_depth, top_parent=self),
            md.DeFire(2 * ef_start, squeeze_start, int(conv_starts / 2), top_parent=self),
            nn.Dropout2d(),
        )

    def forward(self, x):
        shape = x.shape
        over = shape[-1] % self.reduce
        if over:
            over = self.reduce - over
            x = F.pad(x, (int(over / 2), int(over / 2), 0, 0), 'replicate')
        pre_add = self.start(x)
        insides = self.rest(pre_add)
        result = pre_add + insides
        return result


class SqueezeSegBoneRaw(nn.Module):
    def __init__(self, input_channels, squeeze_depth=2, cam_depth=1, conv_starts=64, squeeze_start=16, ef_start=64):
        super().__init__()
        self.reduce = 1
        self.start = nn.Sequential(
            md.Conv(input_channels, conv_starts, 3, 1, 2, top_parent=self),
            md.ContextAggregation(conv_starts, top_parent=self),
            md.Conv(conv_starts, conv_starts, 1, top_parent=self),
        )
        self.pool = md.Pool(3, 2, 1, top_parent=self)
        self.squeeze = md.SqueezePartRaw(conv_starts, squeeze_start, ef_start, squeeze_depth, cam_depth,
                                         top_parent=self)
        self.defire = md.DeFire(2 * ef_start, squeeze_start, int(conv_starts / 2), top_parent=self)
        self.dropout = nn.Dropout2d()

    def forward(self, x):
        shape = x.shape
        over = shape[-1] % self.reduce
        if over:
            over = self.reduce - over
            x = F.pad(x, (int(over / 2), int(over / 2), 0, 0), 'replicate')
        pre_add = self.start(x)
        x = self.pool(pre_add)
        feat, x = self.squeeze(x)
        x = self.defire(x)
        insides = self.dropout(x)
        result = pre_add + insides
        return feat, result
