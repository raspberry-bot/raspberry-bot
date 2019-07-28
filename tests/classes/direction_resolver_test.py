from expects import equal, expect
from api.classes.direction_resolver import DirectionResolver


def resolve(left_motor_speed, right_motor_speed):
    return {
        'left_motor_speed': left_motor_speed,
        'right_motor_speed': right_motor_speed
    }


def test_forward():
    speeds = resolve(50, 50)
    expect(DirectionResolver().resolve(speeds)).to(equal('F'))


def test_forward_right():
    speeds = resolve(70, 50)
    expect(DirectionResolver().resolve(speeds)).to(equal('FR'))


def test_forward_right_with_zero():
    speeds = resolve(70, 0)
    expect(DirectionResolver().resolve(speeds)).to(equal('FR'))


def test_forward_left():
    speeds = resolve(50, 70)
    expect(DirectionResolver().resolve(speeds)).to(equal('FL'))


def test_forward_left_with_zero():
    speeds = resolve(0, 50)
    expect(DirectionResolver().resolve(speeds)).to(equal('FL'))


def test_reverse():
    speeds = resolve(-50, -50)
    expect(DirectionResolver().resolve(speeds)).to(equal('Rv'))


def test_reverse_right():
    speeds = resolve(-70, -50)
    expect(DirectionResolver().resolve(speeds)).to(equal('RvR'))


def test_reverse_right_with_zero():
    speeds = resolve(-70, 0)
    expect(DirectionResolver().resolve(speeds)).to(equal('RvR'))


def test_reverse_left():
    speeds = resolve(-50, -70)
    expect(DirectionResolver().resolve(speeds)).to(equal('RvL'))


def test_reverse_left_with_zero():
    speeds = resolve(0, -70)
    expect(DirectionResolver().resolve(speeds)).to(equal('RvL'))


def test_right():
    speeds = resolve(50, -50)
    expect(DirectionResolver().resolve(speeds)).to(equal('R'))


def test_left():
    speeds = resolve(-50, 50)
    expect(DirectionResolver().resolve(speeds)).to(equal('L'))


def test_stop():
    speeds = resolve(0, 0)
    expect(DirectionResolver().resolve(speeds)).to(equal('S'))
