#!/usr/bin/env python3

import datetime
import os
import threading

from .map_image_io import occupancy_to_pixels, write_pgm, write_png, write_yaml


def save_occupancy_grid_map(slam_map, map_save_dir, name_prefix='map'):
    """Write OccupancyGrid map to PGM + YAML + PNG and return metadata."""
    if slam_map is None:
        raise ValueError('slam_map is None')

    os.makedirs(map_save_dir, exist_ok=True)
    ts = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    timed_name = f'{name_prefix}_{ts}'
    base_path = os.path.join(map_save_dir, timed_name)

    info = slam_map.info
    width = info.width
    height = info.height
    resolution = info.resolution
    origin_x = info.origin.position.x
    origin_y = info.origin.position.y

    pixels = occupancy_to_pixels(slam_map.data, width, height)

    pgm_path = base_path + '.pgm'
    yaml_path = base_path + '.yaml'
    png_path = base_path + '.png'

    write_pgm(pgm_path, width, height, pixels)
    write_yaml(yaml_path, f'{timed_name}.pgm', resolution, origin_x, origin_y)
    write_png(png_path, width, height, pixels)

    return {
        'name': timed_name,
        'base_path': base_path,
        'pgm_path': pgm_path,
        'yaml_path': yaml_path,
        'png_path': png_path,
        'width': width,
        'height': height,
    }


def save_occupancy_grid_map_async(slam_map, map_save_dir, name_prefix='map', on_success=None, on_error=None):
    """Run save_occupancy_grid_map() in a daemon thread."""

    def _worker():
        try:
            result = save_occupancy_grid_map(slam_map, map_save_dir, name_prefix=name_prefix)
            if on_success is not None:
                on_success(result)
        except Exception as err:
            if on_error is not None:
                on_error(err)

    t = threading.Thread(target=_worker, daemon=True)
    t.start()
    return t
