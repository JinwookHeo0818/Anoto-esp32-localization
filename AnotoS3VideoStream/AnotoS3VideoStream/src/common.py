"""Video replay files and checksummed USB framing. Address decoding is performed by the selected PC C++ / MCU endpoint."""
from __future__ import annotations
import json
import math
from pathlib import Path
import struct
import time
import zlib
import os
from lz4_transport import compress as lz4_compress, ENCODER as LZ4_ENCODER

MAGIC = b'AN5!'
VERSION = 2
HEADER = struct.Struct('<4sBBHIIII')
CONFIG = struct.Struct('<HHffHHBBBBfff')
RESULT = struct.Struct('<IQBB5H6I14f32s96s')
HELLO, START, FRAME, END = 1, 2, 3, 4
FRAME_LZ4 = 5
RESULT_EXTRA = struct.Struct("<4I")
HELLO_OK, START_OK, RESULT_OK, END_OK, ERROR = 129, 130, 131, 132, 255
MAX_PAYLOAD = 240 * 180 + 12

class ProtocolError(RuntimeError):
    pass

def packet(kind: int, seq: int, payload: bytes = b'') -> bytes:
    if not 0 <= seq <= 0xffffffff or len(payload) > MAX_PAYLOAD:
        raise ValueError('Invalid packet sequence/length')
    prefix = struct.pack('<4sBBHII', MAGIC, VERSION, kind, 0, seq, len(payload))
    return prefix + struct.pack('<II', zlib.crc32(prefix), zlib.crc32(payload)) + payload

class Client:
    """Exactly one packet in flight; no automatic reprocessing of a frame after timeout."""
    def __init__(self, stream, timeout: float = 30.0):
        self.stream = stream
        self.timeout = timeout
        self.buffer = bytearray()
        self.last_write_complete = True
        self.last_write_ms = 0.
        self.last_pack_ms = 0.
        self.last_wire_bytes = 0
        self.supports_lz4 = False
        self.extended_result = False
        self.compression_enabled = os.getenv('ANOTO_RAW_USB', '0') != '1'
        self._packed_frames = {}

    def prepare_frames(self, prepared: Path, manifest: dict) -> dict:
        """Pre-pack bytes before the replay clock starts; NEVER decode coordinates.
        Read-only access to prepared input. At most 64 MiB compressed data is
        cached in host RAM. Source CRC is checked both now and at transmission.
        Long videos can fall back to inline packing; its time is reported.
        """
        started = time.perf_counter()
        self._packed_frames.clear()
        if not self.supports_lz4 or not self.compression_enabled:
            return dict(enabled=False, preparation_s=0.)
        total = packed_total = cached_bytes = count = compressed = 0
        size = int(manifest['bytes_per_frame'])
        with (Path(prepared) / 'frames.gray').open('rb') as src:
            for entry in manifest['frames']:
                raw = src.read(size)
                checksum = zlib.crc32(raw)
                if len(raw) != size or checksum != int(entry['crc32']):
                    raise ProtocolError('Input cache CRC mismatch during lossless packing')
                block = lz4_compress(raw)
                use = len(block) + 20 <= int((size + 12) * .90)
                chosen = block if use else None
                self._packed_frames[int(entry['index'])] = (size, checksum, chosen)
                count += 1; compressed += int(use); total += size
                packed_total += len(block) + 8 if use else size
                cached_bytes += len(block) if use else 0
                if cached_bytes >= 64 * 1024 * 1024:
                    break
        return dict(enabled=True, encoder=LZ4_ENCODER, frames_prepared=count,
                    compressed_frames=compressed, input_bytes=total,
                    wire_pixel_bytes=packed_total,
                    preparation_s=time.perf_counter()-started,
                    cached_bytes=cached_bytes,
                    note='Lossless byte packing before playback; no PC coordinate decoding.')

    def _read_packet(self, expected_seq: int, expected_type: int, deadline: float, tick=None):
        while time.perf_counter() < deadline:
            # Consume a ready result BEFORE any GUI rendering, so a slow window
            # repaint cannot postpone the timestamp of an already parsed result.
            start = self.buffer.find(MAGIC)
            if start < 0:
                if len(self.buffer) > 3:
                    del self.buffer[:-3]  # tolerate boot text, never parse it as data
            else:
                if start:
                    del self.buffer[:start]
                if len(self.buffer) >= HEADER.size:
                    magic, version, kind, flags, seq, length, hc, pc = HEADER.unpack_from(self.buffer)
                    if version != VERSION or flags or length > MAX_PAYLOAD or zlib.crc32(self.buffer[:16]) != hc:
                        del self.buffer[0]
                        continue
                    total = HEADER.size + length
                    if len(self.buffer) >= total:
                        payload = bytes(self.buffer[HEADER.size:total])
                        del self.buffer[:total]
                        if zlib.crc32(payload) != pc:
                            raise ProtocolError('Reply CRC mismatch. Stop; re-run to start a fresh session.')
                        if seq != expected_seq and expected_type == HELLO_OK:
                            # Previous run may have stopped while one MCU call was
                            # finishing. Discard that historical reply only during
                            # HELLO; START then resets the session. Never publish it.
                            continue
                        if seq != expected_seq:
                            raise ProtocolError(f'Reply sequence {seq} != {expected_seq}; no stale result accepted.')
                        if kind == ERROR:
                            raise ProtocolError('ESP32: ' + payload.decode('utf-8', errors='replace'))
                        if kind != expected_type:
                            raise ProtocolError(f'Reply type {kind} != {expected_type}')
                        if kind == HELLO_OK:
                            caps = json.loads(payload)
                            self.supports_lz4 = bool(caps.get('supports_lz4_frames', False))
                            self.extended_result = bool(caps.get('supports_extended_result', False))
                        return payload
            waiting = getattr(self.stream, 'in_waiting', 0)
            if not waiting and tick is not None:
                tick()
                waiting = getattr(self.stream, 'in_waiting', 0)
            if time.perf_counter() >= deadline:
                break
            data = self.stream.read(min(4096, max(1, waiting)))
            if data:
                self.buffer.extend(data)
        raise TimeoutError('Reply deadline reached. A pending reply may be collected separately; no automatic retry.')

    def transact(self, kind: int, seq: int, payload: bytes, expected: int,
                 deadline=None, tick=None):
        """One request in flight. An absolute host deadline can stop waiting at clip end.

        This cannot preempt an already running MCU computation. There is no queued
        second frame. On a partial write error the sender closes the port: reconnect
        the board before another run if the device parser has an incomplete packet.
        """
        start = time.perf_counter()
        end = min(start + self.timeout, deadline) if deadline is not None else start + self.timeout
        self.last_pack_ms = 0.
        if kind == START and self.extended_result:
            if len(payload) != CONFIG.size:
                raise ProtocolError('Invalid START payload')
            settings = bytearray(payload); settings[19] |= 0x80
            payload = bytes(settings)
        if kind == FRAME and self.supports_lz4 and self.compression_enabled:
            if len(payload) < 12:
                raise ProtocolError('Invalid FRAME payload')
            raw = payload[12:]
            pack_begin = time.perf_counter()
            checksum = zlib.crc32(raw)
            cached = self._packed_frames.get(struct.unpack_from('<I', payload)[0])
            if cached is not None and cached[:2] == (len(raw), checksum):
                packed = cached[2]
            else:
                packed = lz4_compress(raw)
            # Only use compression when it removes >=10% of bytes, including
            # its length/CRC overhead. Incompressible frames stay raw GRAY8.
            if packed is not None and len(packed) + 20 <= int(len(payload) * .90):
                payload = payload[:12] + struct.pack('<II', len(raw), checksum) + packed
                kind = FRAME_LZ4
            self.last_pack_ms = (time.perf_counter() - pack_begin) * 1000.
        data = packet(kind, seq, payload)
        self.last_wire_bytes = len(data)
        self.last_write_complete = False
        write_start = time.perf_counter()
        # v7.4: one USB request, not 32 synchronous 1-KiB writes. Each
        # FRAME is <= 43236 bytes; firmware RX queue is 48 KiB. Partial
        # writes still advance only by the acknowledged count, never retry
        # an unknown-length write exception. Wire bytes/checksums unchanged.
        offset = 0
        while offset < len(data):
            remaining = end - time.perf_counter()
            if remaining <= 0:
                raise TimeoutError('Write deadline reached; frame may be partial. Reconnect board before next run.')
            if hasattr(self.stream, 'write_timeout'):
                self.stream.write_timeout = max(.001, min(.5, remaining))
            try:
                n = self.stream.write(data[offset:])
            except Exception as exc:
                if time.perf_counter() >= end:
                    raise TimeoutError('Clip/write deadline during partial packet; reconnect board before next run') from exc
                raise
            if n is None or n <= 0 or n > len(data) - offset:
                raise ProtocolError('USB write failed. Reconnect then restart; do not retry a partial FRAME.')
            offset += n
        self.last_write_complete = True
        self.last_write_ms = (time.perf_counter() - write_start) * 1000.
        reply = self._read_packet(seq, expected, end, tick)
        # Caller timestamps a near-deadline reply and labels it post-replay;
        # do not consume a valid packet here and then silently discard it.
        return reply, (time.perf_counter() - start) * 1000.0

    def finish_pending(self, seq: int, expected: int, deadline: float):
        """Read the one request already sent; NEVER write/retry/enqueue another."""
        if not self.last_write_complete:
            raise ProtocolError('Cannot wait for a partial request; reconnect the board.')
        return self._read_packet(seq, expected, deadline)

def parse_result(body: bytes) -> dict:
    if len(body) not in (RESULT.size, RESULT.size + RESULT_EXTRA.size):
        raise ProtocolError(f'Invalid result length {len(body)}')
    extra = RESULT_EXTRA.unpack_from(body, RESULT.size) if len(body) > RESULT.size else None
    v = RESULT.unpack_from(body)
    result = dict(frame=v[0], source_us=v[1], valid=bool(v[2]), reference_supported=bool(v[3]),
                  flags=v[4], blocks=v[5], dots=v[6], windows=v[7], fits=v[8],
                  compute_ms=v[9]/1000, receive_ms=v[10]/1000, crc_ms=v[11]/1000,
                  free_internal=v[12], largest_block=v[13], minimum_since_boot=v[14],
                  x_mm=v[15], y_mm=v[16], heading_deg=v[17],
                  dots_ms=v[18], geometry_ms=v[19], address_ms=v[20],
                  box=[[v[21 + 2*i], v[22 + 2*i]] for i in range(4)],
                  status=v[29].split(b'\0',1)[0].decode('ascii',errors='replace'),
                  reason=v[30].split(b'\0',1)[0].decode('utf-8',errors='replace'))
    if extra is not None:
        result.update(unpack_ms=extra[0]/1000, wire_payload_bytes=extra[1],
                      image_seed_ms=extra[2]/1000, image_seed_attempts=extra[3],
                      used_image_phase=bool(result['flags'] & 1024))
        result['mcu_work_ms'] = result['compute_ms'] + result['unpack_ms'] + result['crc_ms']
    if result['valid'] and not all(math.isfinite(result[k]) for k in ('x_mm','y_mm','heading_deg')):
        raise ProtocolError('Device returned a valid flag with non-finite pose')
    if not result['valid']:
        result.update(x_mm=None,y_mm=None,heading_deg=None,box=None)
    return result

def load_manifest(folder: Path) -> dict:
    folder = Path(folder)
    m = json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
    w,h,n = (int(m[k]) for k in ('width','height','frame_count'))
    if m.get('format') != 'ANOTO_GRAY8_V1' or not 32<=w<=240 or not 32<=h<=180 or n<1:
        raise ValueError('Unsupported/corrupt manifest; run with --reprepare')
    frames = m['frames']
    if len(frames)!=n or any(int(f['index'])!=i for i,f in enumerate(frames)):
        raise ValueError('Manifest frame index mismatch')
    times = [int(f['time_us']) for f in frames]
    if times[0]<0 or any(b<=a for a,b in zip(times,times[1:])):
        raise ValueError('Source timestamps must increase')
    if (folder/'frames.gray').stat().st_size!=w*h*n:
        raise ValueError('Truncated frames.gray; regenerate the preparation folder')
    if not math.isfinite(float(m['source_fps'])) or not .1<=float(m['source_fps'])<=1000:
        raise ValueError('Invalid source FPS')
    duration = float(m['source_duration_s'])
    if not math.isfinite(duration) or duration <= times[-1]/1e6 or times[0] != 0:
        raise ValueError('Invalid source duration/start; regenerate prepared video')
    if int(m.get('bytes_per_frame', 0)) != w*h:
        raise ValueError('Manifest bytes_per_frame mismatch')
    return m

def new_output(path: Path) -> Path:
    path = Path(path).resolve()
    if not path.exists():
        path.mkdir(parents=True)
        return path
    for i in range(2,10000):
        p=path.with_name(path.name+f'_{i}')
        if not p.exists():
            p.mkdir(parents=True)
            return p
    raise OSError('Cannot create output folder')

def write_json(path: Path, obj):
    path.write_text(json.dumps(obj,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
