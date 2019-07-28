from expects import equal, expect

from api.classes.motor_speed_calculator import MotorSpeedCalculator
from api.classes import constants


msc = MotorSpeedCalculator()
lm = msc.left_motor
rm = msc.right_motor


def test_equalise_speed_max():
    lm.speed = 100
    rm.speed = -50

    msc.equalise_speed('max')

    expect(lm.speed).to(equal(100))
    expect(rm.speed).to(equal(100))


def test_equalise_speed_min():
    lm.speed = 100
    rm.speed = -50

    msc.equalise_speed('min')

    expect(lm.speed).to(equal(-50))
    expect(rm.speed).to(equal(-50))


def test_increase_forward_speed():
    lm.speed = 100
    rm.speed = -50

    msc.increase_forward_speed()

    expect(lm.speed).to(equal(110))
    expect(rm.speed).to(equal(-40))


def test_decrease_forward_speed():
    lm.speed = 100
    rm.speed = 50

    msc.decrease_forward_speed()

    expect(lm.speed).to(equal(90))
    expect(rm.speed).to(equal(40))


def test_increase_reverse_speed():
    lm.speed = -90
    rm.speed = -50

    msc.increase_reverse_speed()

    expect(lm.speed).to(equal(-100))
    expect(rm.speed).to(equal(-60))


def test_decrease_reverse_speed():
    lm.speed = -90
    rm.speed = -50

    msc.decrease_reverse_speed()

    expect(lm.speed).to(equal(-80))
    expect(rm.speed).to(equal(-40))


def test_move_more_forward_right():
    lm.speed = 100
    rm.speed = 50

    msc.move_more_forward_right()

    expect(lm.speed).to(equal(105))
    expect(rm.speed).to(equal(45))


def test_move_more_forward_left():
    lm.speed = 50
    rm.speed = 100

    msc.move_more_forward_left()

    expect(lm.speed).to(equal(45))
    expect(rm.speed).to(equal(105))


def test_move_more_reverse_left():
    lm.speed = -50
    rm.speed = -100

    msc.move_more_reverse_left()

    expect(lm.speed).to(equal(-45))
    expect(rm.speed).to(equal(-105))


def test_move_more_reverse_right():
    lm.speed = -100
    rm.speed = -50

    msc.move_more_reverse_right()

    expect(lm.speed).to(equal(-105))
    expect(rm.speed).to(equal(-45))


def test_move_more_inplace_left():
    lm.speed = -50
    rm.speed = 50

    msc.move_more_inplace_left()

    expect(lm.speed).to(equal(-60))
    expect(rm.speed).to(equal(60))


def test_move_more_inplace_right():
    lm.speed = 50
    rm.speed = -50

    msc.move_more_inplace_right()

    expect(lm.speed).to(equal(60))
    expect(rm.speed).to(equal(-60))


def test_move_less_inplace_left():
    lm.speed = -50
    rm.speed = 50

    msc.move_less_inplace_left()

    expect(lm.speed).to(equal(-40))
    expect(rm.speed).to(equal(40))


def test_move_less_inplace_right():
    lm.speed = 50
    rm.speed = -50

    msc.move_less_inplace_right()

    expect(lm.speed).to(equal(40))
    expect(rm.speed).to(equal(-40))


def test_reverse():
    lm.speed = 50
    rm.speed = -50

    msc.reverse()

    expect(lm.speed).to(equal(-10))
    expect(rm.speed).to(equal(-10))


def test_move_forward():
    lm.speed = 50
    rm.speed = -50

    msc.move_forward()

    expect(lm.speed).to(equal(10))
    expect(rm.speed).to(equal(10))


def test_move_inplace_left():
    lm.speed = 50
    rm.speed = -50

    msc.move_inplace_left()

    expect(lm.speed).to(equal(-10))
    expect(rm.speed).to(equal(10))


def test_move_inplace_right():
    lm.speed = 50
    rm.speed = -50

    msc.move_inplace_right()

    expect(lm.speed).to(equal(10))
    expect(rm.speed).to(equal(-10))
