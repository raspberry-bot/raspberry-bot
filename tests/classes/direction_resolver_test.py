from expects import equal, expect
from api.classes.direction_resolver import DirectionResolver


def test_forward():
    expect(DirectionResolver().resolve(50, 50)).to(equal('F'))


def test_forward_right():
    expect(DirectionResolver().resolve(70, 50)).to(equal('FR'))


def test_forward_right_with_zero():
    expect(DirectionResolver().resolve(70, 0)).to(equal('FR'))


def test_forward_left():
    expect(DirectionResolver().resolve(50, 70)).to(equal('FL'))


def test_forward_left_with_zero():
    expect(DirectionResolver().resolve(0, 50)).to(equal('FL'))


def test_reverse():
    expect(DirectionResolver().resolve(-50, -50)).to(equal('Rv'))


def test_reverse_right():
    expect(DirectionResolver().resolve(-70, -50)).to(equal('RvR'))


def test_reverse_right_with_zero():
    expect(DirectionResolver().resolve(-70, 0)).to(equal('RvR'))


def test_reverse_left():
    expect(DirectionResolver().resolve(-50, -70)).to(equal('RvL'))


def test_reverse_left_with_zero():
    expect(DirectionResolver().resolve(0, -70)).to(equal('RvL'))


def test_right():
    expect(DirectionResolver().resolve(50, -50)).to(equal('R'))


def test_left():
    expect(DirectionResolver().resolve(-50, 50)).to(equal('L'))


def test_stop():
    expect(DirectionResolver().resolve(0, 0)).to(equal('S'))
