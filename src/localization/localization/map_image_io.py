#!/usr/bin/env python3

import struct
import zlib


def occupancy_to_pixels(occupancy_data, width, height):
    """Convert OccupancyGrid data to black/white export pixels.

    Color mapping:
    - occupied -> 0 (black)
    - free -> 254 (white)
    - unknown/intermediate -> 254 (white)
    """
    occupied_threshold = 65
    free_threshold = 25

    pixels = bytearray(width * height)
    for row in range(height):
        for col in range(width):
            idx = (height - 1 - row) * width + col
            v = occupancy_data[idx]
            if v < 0:
                pixels[row * width + col] = 254
            elif v <= free_threshold:
                pixels[row * width + col] = 254
            elif v >= occupied_threshold:
                pixels[row * width + col] = 0
            else:
                pixels[row * width + col] = 254
    return pixels


def write_pgm(path, width, height, pixels):
    with open(path, 'wb') as f:
        f.write(f'P5\n# CREATOR: fea_slam\n{width} {height}\n255\n'.encode('ascii'))
        f.write(bytes(pixels))


def write_yaml(path, image_filename, resolution, origin_x, origin_y, origin_yaw):
    with open(path, 'w', encoding='ascii') as f:
        f.write(f'image: {image_filename}\n')
        f.write(f'resolution: {resolution}\n')
        f.write(f'origin: [{origin_x:.6f}, {origin_y:.6f}, {origin_yaw:.6f}]\n')
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.25\n')


def build_png_bytes(width, height, pixels):
    def _png_chunk(tag, data):
        checksum = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', checksum)

    ihdr = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    raw_rows = b''.join(b'\x00' + bytes(pixels[r * width:(r + 1) * width]) for r in range(height))
    idat = zlib.compress(raw_rows, 6)
    return b'\x89PNG\r\n\x1a\n' + _png_chunk(b'IHDR', ihdr) + _png_chunk(b'IDAT', idat) + _png_chunk(b'IEND', b'')


def write_png(path, width, height, pixels):
    with open(path, 'wb') as f:
        f.write(build_png_bytes(width, height, pixels))
