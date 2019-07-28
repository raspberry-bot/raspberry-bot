from expects import equal, expect

from api.classes.motor import Motor


pins = {'forward': 20, 'reverse': 21, 'control': 12}
motor = Motor(pins)


def test_clip_positive():
    speed = motor.move(110)
    expect(speed).to(equal(100))


def test_clip_negative():
    speed = motor.move(-110)
    expect(speed).to(equal(-100))


def test_no_clip_positive():
    speed = motor.move(80)
    expect(speed).to(equal(80))


def test_no_clip_negative():
    speed = motor.move(-80)
    expect(speed).to(equal(-80))
