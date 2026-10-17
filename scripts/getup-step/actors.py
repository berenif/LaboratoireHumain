"""Safe existing GetUp binary reader and full-raw parity implementation."""
from dataclasses import dataclass
import struct
from pathlib import Path
import numpy as np

@dataclass
class DenseActor:
    dimensions: tuple
    layers: list

    @classmethod
    def from_bytes(cls, blob):
        if blob[:8] != b'LHGPW001' or len(blob) < 24:
            raise ValueError('invalid actor header')
        dims = struct.unpack('<4I', blob[8:24])
        if dims[0] not in (68, 118) or dims[1:] != (1024, 1024, 42):
            raise ValueError('unsupported source actor architecture')
        expected = sum((a + 1) * b for a, b in zip(dims, dims[1:]))
        if len(blob) != 24 + 4 * expected:
            raise ValueError('invalid actor length')
        values = np.frombuffer(blob, dtype='<f4', offset=24)
        if not np.isfinite(values).all():
            raise ValueError('nonfinite actor')
        offset = 0
        layers = []
        for a, b in zip(dims, dims[1:]):
            weights = values[offset:offset + a*b].reshape(b, a).copy(); offset += a*b
            bias = values[offset:offset+b].copy(); offset += b
            layers.append((weights, bias))
        return cls(dims, layers)

    @classmethod
    def load(cls, path):
        return cls.from_bytes(Path(path).read_bytes())

    def infer(self, observation):
        with np.errstate(over='ignore', invalid='ignore'):
            x = np.asarray(observation, dtype=np.float32)
        if x.shape != (self.dimensions[0],) or not np.isfinite(x).all() or not 0 <= x[-1] <= 1:
            raise ValueError('invalid actor observation')
        power = x[-1]
        for i, (w, b) in enumerate(self.layers):
            # Match native wide accumulation, then the same float32 layer boundary.
            with np.errstate(over='ignore', invalid='ignore'):
                x = (w.astype(np.float64) @ x.astype(np.float64) + b.astype(np.float64)).astype(np.float32)
            if not np.isfinite(x).all():
                raise ValueError('nonfinite actor layer output')
            if i < 2:
                x = np.maximum(x, np.float32(0.))
        return dict(raw=x, action=np.tanh(x[:21]) * power)

    def conditioned_teacher(self):
        if self.dimensions[0] != 68:
            raise ValueError('initializer must be original teacher68')
        layers = [(w.copy(), b.copy()) for w, b in self.layers]
        old, bias = layers[0]
        new = np.zeros((1024, 118), dtype=np.float32)
        new[:, :67] = old[:, :67]
        new[:, 117] = old[:, 67]
        layers[0] = (new, bias)
        return DenseActor((118, 1024, 1024, 42), layers)
