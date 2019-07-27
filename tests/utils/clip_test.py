from expects import equal, expect
from api.utils.clip import _clip


def test_clip_max():
    expect(_clip(101)).to(equal(100))


def test_clip_min():
    expect(_clip(-1)).to(equal(0))


def test_unclipped():
    expect(_clip(1)).to(equal(1))
