"""Bounded-I/O helpers for saved-data audits; no runtime or engine imports."""
import hashlib
import json
from pathlib import Path


CHUNK_SIZE = 1024 * 1024


def digest(path):
    """SHA-256 of the original bytes, with no whole-file allocation."""
    result = hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda: source.read(CHUNK_SIZE), b''):
            result.update(block)
    return result.hexdigest()


def json_array_items(path, chunk_size=CHUNK_SIZE):
    """Yield every JSON-array item, retaining only a chunk and the current item.

    A single item may span chunks. Closing brackets and trailing whitespace are
    also consumed with bounded reads, and truncated or malformed journals fail.
    """
    if chunk_size <= 0:
        raise ValueError('chunk_size must be positive')
    decoder = json.JSONDecoder()
    buffer, position, eof = '', 0, False
    with Path(path).open(encoding='utf-8') as source:
        def refill():
            nonlocal buffer, position, eof
            block = source.read(chunk_size)
            buffer = buffer[position:] + block
            position = 0
            eof = not block

        def skip_space():
            nonlocal position
            while True:
                while position < len(buffer) and buffer[position] in ' \t\r\n':
                    position += 1
                if position < len(buffer) or eof:
                    return
                refill()

        skip_space()
        if position >= len(buffer) or buffer[position] != '[':
            raise ValueError('control journal must be one JSON array')
        position += 1
        skip_space()
        empty = position < len(buffer) and buffer[position] == ']'
        if not empty:
            while True:
                skip_space()
                while True:
                    try:
                        item, end = decoder.raw_decode(buffer, position)
                    except json.JSONDecodeError as error:
                        if eof:
                            raise ValueError('incomplete or invalid control journal') from error
                        refill()
                        continue
                    # A chunk can end inside a number ("1", "1e", "1.").
                    # Do not yield its prematurely decoded prefix.
                    numeric = isinstance(item, (int, float)) and not isinstance(item, bool)
                    partial_number = (end < len(buffer) and numeric
                        and buffer[end] in '.eE+-0123456789')
                    if not eof and (end == len(buffer) or partial_number):
                        refill()
                        continue
                    break
                position = end
                yield item
                skip_space()
                if position >= len(buffer):
                    raise ValueError('incomplete control journal')
                if buffer[position] == ']':
                    break
                if buffer[position] != ',':
                    raise ValueError('invalid control journal separator')
                position += 1
        position += 1
        skip_space()
        if position != len(buffer):
            raise ValueError('trailing data after control journal')
