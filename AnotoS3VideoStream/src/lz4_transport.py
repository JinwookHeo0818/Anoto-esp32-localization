"""Lossless LZ4 block encoder. Optional C extension, dependency-free fallback.
Never estimates dots, geometry, or coordinates. Pixel bytes are unchanged after
decoding. Independent blocks require no previous frame and no resend.
"""
from __future__ import annotations
import struct
try:
    from lz4.block import compress as _native_compress
except ImportError:
    _native_compress = None

def _length(out: bytearray, n: int) -> None:
    while n >= 255:
        out.append(255); n -= 255
    out.append(n)

def compress_python(data: bytes) -> bytes:
    n = len(data); out = bytearray(); table: dict[bytes, int] = {}
    anchor = i = 0
    # LZ4 block format: leave >=5 last literals; final match starts >=12
    # bytes before EOF. Hash/dictionary search only affects compression ratio.
    while i <= n - 12:
        key = data[i:i + 4]; previous = table.get(key, -1); table[key] = i
        if previous < 0 or i - previous > 65535:
            i += 1; continue
        end = i + 4
        while end < n - 5 and data[end] == data[previous + end - i]:
            end += 1
        literals = i - anchor; match = end - i - 4
        out.append((min(literals, 15) << 4) | min(match, 15))
        if literals >= 15: _length(out, literals - 15)
        out.extend(data[anchor:i]); out.extend(struct.pack('<H', i - previous))
        if match >= 15: _length(out, match - 15)
        # Prime nearby keys for the next match, without a per-byte table update
        # for a long constant run.
        for j in range(max(i + 1, end - 3), end):
            table[data[j:j + 4]] = j
        i = anchor = end
    literals = n - anchor
    out.append(min(literals, 15) << 4)
    if literals >= 15: _length(out, literals - 15)
    out.extend(data[anchor:])
    return bytes(out)

def compress(data: bytes) -> bytes:
    if _native_compress is not None:
        return _native_compress(data, mode='fast', acceleration=1, store_size=False)
    return compress_python(data)

ENCODER = 'lz4-C' if _native_compress is not None else 'lz4-python'
