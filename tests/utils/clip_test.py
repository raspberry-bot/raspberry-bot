from expects import equal, expect
from api.utils.clip import clip


def test_clip_max():
    expect(clip(101)).to(equal(100))


def test_clip_min():
    expect(clip(-1)).to(equal(0))


def test_unclipped():
    expect(clip(1)).to(equal(1))
