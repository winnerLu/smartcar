"""Verify that printable tags encode the intended official tag36h11 IDs."""

import importlib.util
from pathlib import Path


ROOT = Path(__file__).parents[1] / 'AprilTag_tag36h11_ID0'
DETECTOR_SOURCE = Path(__file__).parents[1] / 'src' / 'apriltag_detector_node.cpp'


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


TAGS = load_module('gen_8tags', ROOT / 'gen_8tags.py')
BOARD = load_module('gen_a3_pdf', ROOT / 'gen_a3_pdf.py')

OFFICIAL_FIRST_EIGHT = (
    0x0D7E00984B,
    0x0DDA664CA7,
    0x0DC4A1C821,
    0x0E17B470E9,
    0x0EF91D01B1,
    0x0F429CDD73,
    0x005DA29225,
    0x01106CBA43,
)


def decode_generated_tag(path):
    width, height, pixels = BOARD.read_png(path)
    assert width == height
    cell = width // TAGS.TOTAL
    code = 0
    for x, y in zip(TAGS.BIT_X, TAGS.BIT_Y):
        sample_x = (x + 1) * cell + cell // 2
        sample_y = (y + 1) * cell + cell // 2
        code = (code << 1) | (
            pixels[sample_y * width + sample_x] > 127)
    return code


def test_codewords_and_print_scale_match_detector_configuration():
    assert tuple(TAGS.CODES[index] for index in range(8)) == OFFICIAL_FIRST_EIGHT
    assert BOARD.TAG_BLACK_MM == 50
    assert BOARD.TAG_TILE_MM == 62.5

    for tag_id, expected in enumerate(OFFICIAL_FIRST_EIGHT):
        path = ROOT / 'tags_5cm' / f'tag36h11_id{tag_id:02d}.png'
        assert decode_generated_tag(path) == expected


def test_detector_corner_order_uses_image_down_as_positive_board_y():
    """Keep the solvePnP tag axes consistent with the printed board layout.

    apriltag_detection_t::p starts at the lower-left corner.  Mapping that
    corner to (-x, +y) makes the solved +y direction point down in the image,
    which is also how TAG_POSITIONS_BOARD and board_tag_offset() define +y.
    """
    source = DETECTOR_SOURCE.read_text(encoding='utf-8')
    expected_mapping = (
        '{-half,  half, 0.0f}, {half,  half, 0.0f},\n'
        '        {half, -half, 0.0f}, {-half, -half, 0.0f},'
    )
    assert expected_mapping in source
