import os
from subprocess import call


def shutdown():
    call(["sudo", "shutdown", "-h", "now"])

def restart():
    call(['sudo', 'reboot'])

def info():
    return os.uname()


system_cmds = {
    'shutdown': shutdown,
    'restart': restart,
    'info': info,
}