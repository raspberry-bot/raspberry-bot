from expects import equal, expect

from api.classes.motor_speed_resolver import MotorSpeedResolver
from api.classes import constants


################################################
# FORWARD
################################################

def test_direction_forward_to_forward():
    state = {
        'current_direction': constants.DIRECTION_FORWARD,
        'left_motor_speed': 50,
        'right_motor_speed': 50
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_FORWARD)).to(equal({
        'left_motor_speed': 60,
        'right_motor_speed': 60
    }))


def test_direction_forward_to_reverse():
    state = {
        'current_direction': constants.DIRECTION_FORWARD,
        'left_motor_speed': 50,
        'right_motor_speed': 50
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_REVERSE)).to(equal({
        'left_motor_speed': 40,
        'right_motor_speed': 40
    }))


def test_direction_forward_to_right():
    state = {
        'current_direction': constants.DIRECTION_FORWARD,
        'left_motor_speed': 50,
        'right_motor_speed': 50
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_RIGHT)).to(equal({
        'left_motor_speed': 55,
        'right_motor_speed': 45
    }))


def test_direction_forward_to_left():
    state = {
        'current_direction': constants.DIRECTION_FORWARD,
        'left_motor_speed': 50,
        'right_motor_speed': 50
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_LEFT)).to(equal({
        'left_motor_speed': 45,
        'right_motor_speed': 55
    }))

################################################
# REVERSE
################################################


def test_direction_reverse_to_forward():
    state = {
        'current_direction': constants.DIRECTION_REVERSE,
        'left_motor_speed': -50,
        'right_motor_speed': -50
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_FORWARD)).to(equal({
        'left_motor_speed': -40,
        'right_motor_speed': -40
    }))


def test_direction_reverse_to_reverse():
    state = {
        'current_direction': constants.DIRECTION_REVERSE,
        'left_motor_speed': -50,
        'right_motor_speed': -50
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_REVERSE)).to(equal({
        'left_motor_speed': -60,
        'right_motor_speed': -60
    }))


def test_direction_reverse_to_right():
    state = {
        'current_direction': constants.DIRECTION_REVERSE,
        'left_motor_speed': -50,
        'right_motor_speed': -50
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_RIGHT)).to(equal({
        'left_motor_speed': -55,
        'right_motor_speed': -45
    }))


def test_direction_reverse_to_left():
    state = {
        'current_direction': constants.DIRECTION_REVERSE,
        'left_motor_speed': -50,
        'right_motor_speed': -50
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_LEFT)).to(equal({
        'left_motor_speed': -45,
        'right_motor_speed': -55
    }))

################################################
# RIGHT
################################################


def test_direction_right_to_forward():
    state = {
        'current_direction': constants.DIRECTION_RIGHT,
        'left_motor_speed': 50,
        'right_motor_speed': -50
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_FORWARD)).to(equal({
        'left_motor_speed': 50,
        'right_motor_speed': 50
    }))


def test_direction_right_to_reverse():
    state = {
        'current_direction': constants.DIRECTION_RIGHT,
        'left_motor_speed': 50,
        'right_motor_speed': -50
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_REVERSE)).to(equal({
        'left_motor_speed': -50,
        'right_motor_speed': -50
    }))


def test_direction_right_to_right():
    state = {
        'current_direction': constants.DIRECTION_RIGHT,
        'left_motor_speed': 50,
        'right_motor_speed': -50
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_RIGHT)).to(equal({
        'left_motor_speed': 60,
        'right_motor_speed': -60
    }))


def test_direction_right_to_left():
    state = {
        'current_direction': constants.DIRECTION_RIGHT,
        'left_motor_speed': 50,
        'right_motor_speed': -50
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_LEFT)).to(equal({
        'left_motor_speed': 40,
        'right_motor_speed': -40
    }))

################################################
# LEFT
################################################


def test_direction_left_to_forward():
    state = {
        'current_direction': constants.DIRECTION_LEFT,
        'left_motor_speed': -50,
        'right_motor_speed': 50
    }
    # TODO
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_FORWARD)).to(equal({
        'left_motor_speed': 50,
        'right_motor_speed': 50
    }))


def test_direction_left_to_reverse():
    state = {
        'current_direction': constants.DIRECTION_LEFT,
        'left_motor_speed': -50,
        'right_motor_speed': 50
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_REVERSE)).to(equal({
        'left_motor_speed': -50,
        'right_motor_speed': -50
    }))


def test_direction_left_to_right():
    state = {
        'current_direction': constants.DIRECTION_LEFT,
        'left_motor_speed': -50,
        'right_motor_speed': 50
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_RIGHT)).to(equal({
        'left_motor_speed': -40,
        'right_motor_speed': 40
    }))


def test_direction_left_to_left():
    state = {
        'current_direction': constants.DIRECTION_LEFT,
        'left_motor_speed': -50,
        'right_motor_speed': 50
    }
    # TODO
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_LEFT)).to(equal({
        'left_motor_speed': -60,
        'right_motor_speed': 60
    }))

################################################
# FORWARD RIGHT
################################################


def test_direction_forward_right_to_forward():
    state = {
        'current_direction': constants.DIRECTION_FORWARD_RIGHT,
        'left_motor_speed': 50,
        'right_motor_speed': 40
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_FORWARD)).to(equal({
        'left_motor_speed': 50,
        'right_motor_speed': 50
    }))


def test_direction_forward_right_to_reverse():
    state = {
        'current_direction': constants.DIRECTION_FORWARD_RIGHT,
        'left_motor_speed': 50,
        'right_motor_speed': 40
    }
    # TODO
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_REVERSE)).to(equal({
        'left_motor_speed': 40,
        'right_motor_speed': 30
    }))


def test_direction_forward_right_to_right():
    state = {
        'current_direction': constants.DIRECTION_FORWARD_RIGHT,
        'left_motor_speed': 50,
        'right_motor_speed': 40
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_RIGHT)).to(equal({
        'left_motor_speed': 55,
        'right_motor_speed': 35
    }))


def test_direction_forward_right_to_left():
    state = {
        'current_direction': constants.DIRECTION_FORWARD_RIGHT,
        'left_motor_speed': 50,
        'right_motor_speed': 40
    }
    # TODO
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_LEFT)).to(equal({
        'left_motor_speed': 45,
        'right_motor_speed': 45
    }))

################################################
# FORWARD LEFT
################################################


def test_direction_forward_left_to_forward():
    state = {
        'current_direction': constants.DIRECTION_FORWARD_LEFT,
        'left_motor_speed': 40,
        'right_motor_speed': 60
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_FORWARD)).to(equal({
        'left_motor_speed': 60,
        'right_motor_speed': 60
    }))


def test_direction_forward_left_to_reverse():
    state = {
        'current_direction': constants.DIRECTION_FORWARD_LEFT,
        'left_motor_speed': 40,
        'right_motor_speed': 60
    }
    # TODO
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_REVERSE)).to(equal({
        'left_motor_speed': 30,
        'right_motor_speed': 50
    }))


def test_direction_forward_left_to_right():
    state = {
        'current_direction': constants.DIRECTION_FORWARD_LEFT,
        'left_motor_speed': 40,
        'right_motor_speed': 60
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_RIGHT)).to(equal({
        'left_motor_speed': 45,
        'right_motor_speed': 55
    }))


def test_direction_forward_left_to_left():
    state = {
        'current_direction': constants.DIRECTION_FORWARD_LEFT,
        'left_motor_speed': 40,
        'right_motor_speed': 60
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_LEFT)).to(equal({
        'left_motor_speed': 35,
        'right_motor_speed': 65
    }))

################################################
# REVERSE LEFT
################################################


def test_direction_reverse_left_to_forward():
    state = {
        'current_direction': constants.DIRECTION_REVERSE_LEFT,
        'left_motor_speed': -50,
        'right_motor_speed': -80
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_FORWARD)).to(equal({
        'left_motor_speed': -40,
        'right_motor_speed': -70
    }))


def test_direction_reverse_left_to_reverse():
    state = {
        'current_direction': constants.DIRECTION_REVERSE_LEFT,
        'left_motor_speed': -50,
        'right_motor_speed': -80
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_REVERSE)).to(equal({
        'left_motor_speed': -80,
        'right_motor_speed': -80
    }))


def test_direction_reverse_left_to_right():
    state = {
        'current_direction': constants.DIRECTION_REVERSE_LEFT,
        'left_motor_speed': -50,
        'right_motor_speed': -80
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_RIGHT)).to(equal({
        'left_motor_speed': -55,
        'right_motor_speed': -75
    }))


def test_direction_reverse_left_to_left():
    state = {
        'current_direction': constants.DIRECTION_REVERSE_LEFT,
        'left_motor_speed': -50,
        'right_motor_speed': -80
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_LEFT)).to(equal({
        'left_motor_speed': -45,
        'right_motor_speed': -85
    }))

################################################
# REVERSE RIGHT
################################################


def test_direction_reverse_right_to_forward():
    state = {
        'current_direction': constants.DIRECTION_REVERSE_RIGHT,
        'left_motor_speed': -80,
        'right_motor_speed': -50
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_FORWARD)).to(equal({
        'left_motor_speed': -70,
        'right_motor_speed': -40
    }))


def test_direction_reverse_right_to_reverse():
    state = {
        'current_direction': constants.DIRECTION_REVERSE_RIGHT,
        'left_motor_speed': -80,
        'right_motor_speed': -50
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_REVERSE)).to(equal({
        'left_motor_speed': -80,
        'right_motor_speed': -80
    }))


def test_direction_reverse_right_to_right():
    state = {
        'current_direction': constants.DIRECTION_REVERSE_RIGHT,
        'left_motor_speed': -80,
        'right_motor_speed': -50
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_RIGHT)).to(equal({
        'left_motor_speed': -85,
        'right_motor_speed': -45
    }))


def test_direction_reverse_right_to_left():
    state = {
        'current_direction': constants.DIRECTION_REVERSE_RIGHT,
        'left_motor_speed': -80,
        'right_motor_speed': -50
    }
    expect(MotorSpeedResolver().resolve(state, constants.TARGET_ACTION_LEFT)).to(equal({
        'left_motor_speed': -75,
        'right_motor_speed': -55
    }))
