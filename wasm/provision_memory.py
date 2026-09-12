#!/usr/bin/env python3
"""Permit fixed memory budgets at instantiation with Emscripten 6.0.9.

ALLOW_MEMORY_GROWTH=0 emits a memory import with maximum == minimum.
Change only that import's maximum to 8192 pages (512 MiB). The wrapper
supplies memory with initial == maximum == its chosen budget. Allocator
code remains compiled without growth. Reject any unexpected binary layout.
"""
from pathlib import Path
import sys


def uleb(value):
    result = bytearray()
    while True:
        byte = value & 127
        value >>= 7
        result.append(byte | (128 if value else 0))
        if not value:
            return result


def read(data, offset):
    value = 0
    for shift in range(0, 35, 7):
        if offset >= len(data):
            raise ValueError('truncated uint32')
        byte = data[offset]
        offset += 1
        if shift == 28 and byte > 15:
            raise ValueError('uint32 overflow')
        value |= (byte & 127) << shift
        if not byte & 128:
            return value, offset
    raise ValueError('invalid uint32')


def name(data, offset):
    size, offset = read(data, offset)
    end = offset + size
    if end > len(data):
        raise ValueError('truncated import name')
    return bytes(data[offset:end]), end


def imports(data):
    count, offset = read(data, 0)
    result = bytearray(uleb(count))
    memories = 0
    if count > len(data):
        raise ValueError('invalid import count')
    for _ in range(count):
        start = offset
        module, offset = name(data, offset)
        field, offset = name(data, offset)
        if offset >= len(data):
            raise ValueError('truncated import')
        kind = data[offset]
        offset += 1
        if kind == 0:  # Function import: type index.
            _, offset = read(data, offset)
            result += data[start:offset]
        elif kind == 2:
            prefix = offset
            flags, offset = read(data, offset)
            minimum, offset = read(data, offset)
            maximum, offset = read(data, offset)
            # Release glue minifies import names; preserve both names verbatim.
            if not module or not field or (flags, minimum, maximum) != (1, 32, 32):
                raise ValueError('expected one non-shared wasm32 memory import with 32/32 pages')
            memories += 1
            result += data[start:prefix] + uleb(1) + uleb(32) + uleb(8192)
        else:
            raise ValueError(f'unexpected import kind {kind}')
    if memories != 1 or offset != len(data):
        raise ValueError('expected exactly one memory and no trailing import data')
    return result


def provision(data):
    if len(data) > 32 * 1024 * 1024 or data[:8] != b'\0asm\x01\0\0\0':
        raise ValueError('expected a WASM v1 binary of at most 32 MiB')
    result = bytearray(data[:8])
    offset, found = 8, False
    while offset < len(data):
        start = offset
        kind = data[offset]
        size, offset = read(data, offset + 1)
        end = offset + size
        if end > len(data):
            raise ValueError('truncated section')
        if kind == 2:
            if found:
                raise ValueError('duplicate import section')
            found = True
            section = imports(data[offset:end])
            result += bytes([kind]) + uleb(len(section)) + section
        else:
            result += data[start:end]
        offset = end
    if not found:
        raise ValueError('missing import section')
    return result


if __name__ == '__main__':
    path = Path(sys.argv[1])
    # Validate and construct completely before replacing the linker output.
    updated = provision(path.read_bytes())
    path.write_bytes(updated)
