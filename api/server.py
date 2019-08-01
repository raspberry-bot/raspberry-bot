import argparse
import json
import os.path
import subprocess
import time
from datetime import datetime

# import netifaces
from wifi import Cell, Scheme

import tornado.httpserver
import tornado.ioloop
import tornado.websocket
from tornado import web
from tornado.options import define, options
from datetime import timedelta

import psutil

import requests

from utils.wifi import WifiManager

# import cv2
# import base64
# import rospy
# from geometry_msgs.msg import Twist

API_SERVER_ROOT = os.environ.get('API_SERVER_ROOT')
BOT_ROOT = os.environ.get('RASPBERRYBOT_ROOT')

EVENTS_FILE = os.path.join(BOT_ROOT, 'logs/events.log')
VERSION_FILE = os.path.join(BOT_ROOT, 'src/VERSION')
CONFIG_FILE = os.path.join(BOT_ROOT, 'src/configs/bot-config.json')
SUPERVISORD_LOGS_PATH = os.path.join(BOT_ROOT, 'supervisor/logs/')

REPO_LATEST_VERSION_URL = 'https://raw.githubusercontent.com/aeldaly/The-Green-Bots/master/VERSION'


class Application(web.Application):
    def __init__(self, model):
        # self.camera = cv2.VideoCapture(0)
        # self.control_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.model = model
        handlers = [
            (r"/api/system", SystemHandler),
            (r"/api/wifi-status", WifiStatusHandler),
            (r"/api/wifi", WifiHandler),
            (r"/api/logs", LogHandler),
            # (r"/api/intelligence", IntelligenceHandler),
            (r"/api/update", UpdateHandler),
            (r"/api/ping", PingHandler)
        ]
        web.Application.__init__(self, handlers, debug=True)


class BaseHandler(web.RequestHandler):
    def set_default_headers(self):
        self.set_header("Access-Control-Allow-Origin", "*")
        self.set_header("Access-Control-Allow-Headers", "x-requested-with")
        self.set_header('Access-Control-Allow-Methods', 'POST, GET, OPTIONS')


def add_event(new_line):
    with open(EVENTS_FILE, 'a+') as events_file:
        events_file.write(datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S.%f") + ' - ' + str(new_line) + '\n')


def update_config_file(new_dict):
    '''
    Schema:
        {
            'wifi': {
                'ssid': '',
                'password': ''
            },
            # 'intelligence': {
            #     'voice-command': True,
            #     'autonomous-driving': True,
            #     'nightvision-camera': True,
            #     'ultrasonic-distance-meter': True,
            # },
            'firmware': {
                'version': '1.0',
                'last_update': ''
            }
        }
    '''
    add_event('Updated Config File...')
    with open(CONFIG_FILE, 'r+') as config_file:
        data = json.load(config_file)
        for k, v in new_dict.items():
            data[k] = v
        config_file.seek(0)
        json.dump(data, config_file, indent=4)
        config_file.truncate()


def get_config_file():
    config = {}
    with open(CONFIG_FILE, 'r') as config_f:
        config = json.load(config_f)
    return config


def cmd(command):
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    log, _err = proc.communicate()
    try:
        log = log.decode("utf-8")
    except Exception as e:
        return e
    else:
        return log


def tail(path, lines=20):
    return cmd(['tail', '-%d' % lines, path])


class UpdateHandler(BaseHandler):

    def get(self):
        config = get_config_file()
        response = requests.get(REPO_LATEST_VERSION_URL)
        latest_version = response.content.decode('utf-8')
        firmware = config.get('firmware', {})
        self.write(json.dumps({
            'new_update_available': float(latest_version) > float(firmware.get('version', 0.0)),
            'latest_version': latest_version,
            'firmware': firmware
        }))

    def post(self):
        data = tornado.escape.json_decode(self.request.body)
        tmp_destination = '/tmp/raspberrybot'
        final_destination = '/opt/raspberrybot/src'
        add_event('Updating The Raspberry Bot local repository...')
        result = cmd([
            'git', 'clone', '-b', data.get('gitbranch', 'master'),
            '--single-branch', '--depth', '1',
            data.get('gitrepo'),
            tmp_destination
        ])
        result += 'Storing new cloned repo on %s\n' % tmp_destination
        result += 'Replacing old repo at %s with new cloned repo from %s\n' % (final_destination, tmp_destination)
        result += 'Restarting supervisord...\n'
        result += 'You need to refresh this page!\n'
        add_event('UPDATING!\n\t' + result)
        self.write(json.dumps(result))
        cmd(['mv', tmp_destination, final_destination])
        update_config_file({
            'firmware': {
                'version': self.get_version(),
                'last_updated': datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            }
        })
        time.sleep(10)
        self.restart_supervisord()

    def get_version(self):
        version_f = open(VERSION_FILE, 'r')
        return version_f.read()
    
    def restart_supervisord(self):
        add_event('Restarting supervisord now ...')
        cmd(['sudo', '/etc/init.d/raspberrybot-api.sh', 'restart'])


class PingHandler(tornado.websocket.WebSocketHandler):
    clients = set()

    def check_origin(self, origin):
        # Allow access from every origin
        return True

    def open(self):
        PingHandler.clients.add(self)
        print("WebSocket opened from: " + self.request.remote_ip)
        
    def on_message(self, message):
        self.write_message(str(int(time.time() * 1000)))

    def on_close(self):
        PingHandler.clients.remove(self)
        print("WebSocket closed from: " + self.request.remote_ip)


class LogHandler(BaseHandler):
    def get(self):
        tornado_logs = ''
        tornado_logs += tail(SUPERVISORD_LOGS_PATH + 'tornado-stdout.log', lines=100)
        tornado_logs += tail(SUPERVISORD_LOGS_PATH + 'tornado-operate-stdout.log', lines=100)
        
        jupyter_logs = ''
        jupyter_logs += tail(SUPERVISORD_LOGS_PATH + 'jupyter-stderr.log', lines=100)
        jupyter_logs += tail(SUPERVISORD_LOGS_PATH + 'jupyter-stdout.log', lines=100)

        supervisord_logs = tail(SUPERVISORD_LOGS_PATH + 'supervisord.log', lines=100)
        
        nginx_logs = ''
        nginx_logs += tail('/var/log/nginx/access.log', lines=100)
        nginx_logs += tail('/var/log/nginx/error.log', lines=100)

        logs = {
            'events': tail(EVENTS_FILE),
            'syslog': tail('/var/log/syslog', lines=100),
            'tornado': tornado_logs,
            'jupyter': jupyter_logs,
            'supervisord': supervisord_logs,
            'nginx': nginx_logs,
        }
        self.write(json.dumps(logs))


class IntelligenceHandler(BaseHandler):
    def get(self):
        config = get_config_file()
        self.write(json.dumps(config.get('intelligence')))

    def post(self):
        data = tornado.escape.json_decode(self.request.body)
        print(data)
        intelligence_conf = {
            'intelligence': {
                'voice-command': data.get('voice-command', False) == 'on',
                'autonomous-driving': data.get('autonomous-driving', False) == 'on',
                'nightvision-camera': data.get('nightvision-camera', False) == 'on',
                'ultrasonic-distance-meter': data.get('ultrasonic-distance-meter', False) == 'on',
            }
        }
        add_event(intelligence_conf)
        update_config_file(intelligence_conf)


class SystemHandler(BaseHandler):
    def get(self):
        config = get_config_file()
        sys_lines = ''
        sys_lines += 'Firmware Version: %s\n' % config.get('firmware', {}).get('version')
        sys_lines += 'Last Updated: %s\n' % config.get('firmware', {}).get('last_updated')
        sys_lines += 'System Uptime: %s\n' % self._uptime()
        sys_lines += 'CPU Usage: %.2f%% \t Memory Usage: %.2f%%\n' % (psutil.cpu_percent(), psutil.virtual_memory().percent)
        sys_lines += '$ uname: \n\t%s' % '\n\t'.join(os.uname())
        self.write(json.dumps(sys_lines))

    def post(self):
        data = tornado.escape.json_decode(self.request.body)
        if data['command'] == 'shutdown':
            self._shutdown()
        elif data['command'] == 'reboot':
            self._reboot()
        elif data['command'] == 'reset_factory':
            self._reset_factory()

    def _uptime(self):
        uptime_string = None
        with open('/proc/uptime', 'r') as f:
            uptime_seconds = float(f.readline().split()[0])
            uptime_string = str(timedelta(seconds=uptime_seconds))
        return uptime_string

    def _shutdown(self):
        add_event('Shutting Down')
        cmd(['shutdown', '-h', 'now'])

    def _reboot(self):
        add_event('Rebooting')
        cmd(['reboot'])

    def _reset_factory(self):
        add_event('Restarting to Factory')
        print('Reset to Factory...')


class WifiStatusHandler(BaseHandler):

    def get(self):
        wlan0 = ('#' * 10) + ' $ iwconfig wlan0 ' + ('#' * 10) + '\n'
        wlan0 += cmd(['iwconfig', 'wlan0'])
        wlan0 += ('#' * 10) + ' $ ifconfig wlan0 ' + ('#' * 10) + '\n'
        wlan0 += cmd(['ifconfig', 'wlan0'])
        self.write(json.dumps(wlan0))


class WifiHandler(BaseHandler):
    WIRELESS_CONFIG_TEMPLATE = '''
country=CA # Your 2-digit country code
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
network={
    ssid="%(selected-ssid)s"
    psk="%(password)s"
    key_mgmt=WPA-PSK
}
'''

    def post(self):
        data = tornado.escape.json_decode(self.request.body)
        add_event('Connecting to WiFi: %s' % data['selected-ssid'])
        update_config_file({
            'wifi': {
                'ssid': data['selected-ssid'],
                'password': data['password'],
            }
        })
        wlan0_was_not_currently_connected = WifiManager.get_currently_connected_ssid() == 'off/any'
        if wlan0_was_not_currently_connected:
            cmd(['sudo', 'ip', 'link', 'set', 'wlan0', 'up'])  # Make sure it's up
        add_event('Trying to connect to wifi using wpa')
        self.configure_wpa(data)
        add_event('Connected to wifi using wpa: %s' % data['selected-ssid'])

    def get(self):
        try:
            wlan0_was_not_currently_connected = WifiManager.get_currently_connected_ssid() == 'off/any'
            if wlan0_was_not_currently_connected:
                cmd(['sudo', 'ip', 'link', 'set', 'wlan0', 'up'])  # Make sure it's up
            ssid_dict = WifiManager.get_dict_of_ssids_with_status()
            self.write(json.dumps(ssid_dict))
        except Exception as ex:
            self.write(json.dumps(str(ex)))

    def generate_wireless_wpa_config(self, data):
        return WifiHandler.WIRELESS_CONFIG_TEMPLATE % data

    def configure_wpa(self, data):
        wireless_config = self.generate_wireless_wpa_config(data)
        with open('/etc/wpa_supplicant/wpa_supplicant.conf', 'w+') as wirelesss_config_f:
            wirelesss_config_f.write(wireless_config)
        cmd(['wpa_cli',])
        cmd(['sudo', 'ifconfig', 'wlan0', 'up'])
        cmd(['sudo', 'systemctl', 'daemon-reload'])
        cmd(['sudo', 'systemctl', 'restart', 'dhcpcd'])
        time.sleep(30)
        # cmd(['sudo', 'reboot'])


def main(args):
    define("port", default=args.port, help="Run on the given port", type=int)
    http_api = tornado.httpserver.HTTPServer(Application({}))
    http_api.listen(options.port)
    tornado.ioloop.IOLoop.instance().start()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Web Interface api')

    parser.add_argument('--port', type=int,
                        help="port to run on. Must be supplied.")
    args = parser.parse_args()
    main(args)
