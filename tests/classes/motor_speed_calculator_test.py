from expects import equal, expect

from api.classes.motor_speed_calculator import MotorSpeedCalculator
from api.classes import constants


def speeds(left_motor_speed, right_motor_speed):
    return {
        'left_motor_speed': left_motor_speed,
        'right_motor_speed': right_motor_speed
    }


def test_equalise_speed_max():
    msc = MotorSpeedCalculator(speeds(100, -50))
    msc.equalise_speed('max')

    lm = msc.left_motor
    rm = msc.right_motor

    expect(lm.speed).to(equal(100))
    expect(rm.speed).to(equal(100))


def test_equalise_speed_min():
    msc = MotorSpeedCalculator(speeds(100, -50))
    msc.equalise_speed('min')

    lm = msc.left_motor
    rm = msc.right_motor

    expect(lm.speed).to(equal(-50))
    expect(rm.speed).to(equal(-50))


def test_increase_forward_speed():
    msc = MotorSpeedCalculator(speeds(90, -50))
    msc.increase_forward_speed()

    lm = msc.left_motor
    rm = msc.right_motor

    expect(lm.speed).to(equal(100))
    expect(rm.speed).to(equal(-40))


def test_decrease_forward_speed():
    msc = MotorSpeedCalculator(speeds(100, 50))
    msc.decrease_forward_speed()

    lm = msc.left_motor
    rm = msc.right_motor

    expect(lm.speed).to(equal(90))
    expect(rm.speed).to(equal(40))


def test_increase_reverse_speed():
    msc = MotorSpeedCalculator(speeds(-90, -50))
    msc.increase_reverse_speed()

    lm = msc.left_motor
    rm = msc.right_motor

    expect(lm.speed).to(equal(-100))
    expect(rm.speed).to(equal(-60))


def test_decrease_reverse_speed():
    msc = MotorSpeedCalculator(speeds(-90, -50))
    msc.decrease_reverse_speed()

    lm = msc.left_motor
    rm = msc.right_motor

    expect(lm.speed).to(equal(-80))
    expect(rm.speed).to(equal(-40))


def test_move_more_forward_right():
    msc = MotorSpeedCalculator(speeds(90, 50))
    msc.move_more_forward_right()

    lm = msc.left_motor
    rm = msc.right_motor

    expect(lm.speed).to(equal(95))
    expect(rm.speed).to(equal(45))


def test_move_more_forward_left():
    msc = MotorSpeedCalculator(speeds(50, 90))
    msc.move_more_forward_left()

    lm = msc.left_motor
    rm = msc.right_motor

    expect(lm.speed).to(equal(45))
    expect(rm.speed).to(equal(95))


def test_move_more_reverse_left():
    msc = MotorSpeedCalculator(speeds(-50, -90))
    msc.move_more_reverse_left()

    lm = msc.left_motor
    rm = msc.right_motor

    expect(lm.speed).to(equal(-45))
    expect(rm.speed).to(equal(-95))


def test_move_more_reverse_right():
    msc = MotorSpeedCalculator(speeds(-90, -50))
    msc.move_more_reverse_right()

    lm = msc.left_motor
    rm = msc.right_motor

    expect(lm.speed).to(equal(-95))
    expect(rm.speed).to(equal(-45))


def test_move_more_inplace_left():
    msc = MotorSpeedCalculator(speeds(-50, 50))
    msc.move_more_inplace_left()

    lm = msc.left_motor
    rm = msc.right_motor

    expect(lm.speed).to(equal(-60))
    expect(rm.speed).to(equal(60))


def test_move_more_inplace_right():
    msc = MotorSpeedCalculator(speeds(50, -50))
    msc.move_more_inplace_right()

    lm = msc.left_motor
    rm = msc.right_motor

    expect(lm.speed).to(equal(60))
    expect(rm.speed).to(equal(-60))


def test_move_less_inplace_left():
    msc = MotorSpeedCalculator(speeds(-50, 50))
    msc.move_less_inplace_left()

    lm = msc.left_motor
    rm = msc.right_motor

    expect(lm.speed).to(equal(-40))
    expect(rm.speed).to(equal(40))


def test_move_less_inplace_right():
    msc = MotorSpeedCalculator(speeds(50, -50))
    msc.move_less_inplace_right()

    lm = msc.left_motor
    rm = msc.right_motor

    expect(lm.speed).to(equal(40))
    expect(rm.speed).to(equal(-40))


def test_reverse():
    msc = MotorSpeedCalculator(speeds(50, -50))
    msc.reverse()

    lm = msc.left_motor
    rm = msc.right_motor

    expect(lm.speed).to(equal(-10))
    expect(rm.speed).to(equal(-10))


def test_move_forward():
    msc = MotorSpeedCalculator(speeds(50, -50))
    msc.move_forward()

    lm = msc.left_motor
    rm = msc.right_motor

    expect(lm.speed).to(equal(10))
    expect(rm.speed).to(equal(10))


def test_move_inplace_left():
    msc = MotorSpeedCalculator(speeds(50, -50))
    msc.move_inplace_left()

    lm = msc.left_motor
    rm = msc.right_motor

    expect(lm.speed).to(equal(-10))
    expect(rm.speed).to(equal(10))


def test_move_inplace_right():
    msc = MotorSpeedCalculator(speeds(50, -50))
    msc.move_inplace_right()

    lm = msc.left_motor
    rm = msc.right_motor

    expect(lm.speed).to(equal(10))
    expect(rm.speed).to(equal(-10))
