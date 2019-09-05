import wifi
import subprocess
import RPi.GPIO as GPIO
import time
import os

class WifiManager:
    @staticmethod
    def get_dict_of_ssids_with_status():
        SSIDs = WifiManager.scan()
        current_ssid = WifiManager.get_currently_connected_ssid()
        ssid_dict = {}
        for ssid in SSIDs:
            if ssid:
                if ssid == current_ssid:
                    ssid_dict[ssid] = 'connected'
                else:
                    ssid_dict[ssid] = 'disconnect'
        
        if WifiAccessPointManager.is_access_point() and 'connected' not in ssid_dict.values():
            ssid_dict['RaspberryBot'] = 'connected'

        return ssid_dict

    @staticmethod
    def get_currently_connected_ssid():
        p = subprocess.Popen("iwconfig wlan0 |grep SSID", stdout=subprocess.PIPE, shell=True)
        (output, err) = p.communicate()
        value = output.decode('utf-8').strip().split('ESSID:')[-1]
        if value != 'off/any':
            return value.replace("\"","")
        return value

    @staticmethod
    def scan():
        return [wifi.ssid for wifi in wifi.Cell.all('wlan0')]

    @staticmethod
    def find_from_search_list(ssid):
        wifilist = WifiManager.scan()
        for cell in wifilist:
            if cell.ssid == ssid:
                return cell
        return False

    @staticmethod
    def find_from_saved_list(ssid):
        cell = wifi.Scheme.find('wlan0', ssid)
        if cell:
            return cell
        return False

    @staticmethod
    def connect(ssid, password=None):
        cell = WifiManager.find_from_search_list(ssid)
        if cell:
            savedcell = WifiManager.find_from_saved_list(cell.ssid)
            # Already Saved from Setting
            if savedcell:
                savedcell.activate()
                return cell
            # First time to conenct
            else:
                if cell.encrypted:
                    if password:
                        scheme = WifiManager.add(cell, password)
                        try:
                            scheme.activate()
                        # Wrong Password
                        except wifi.exceptions.ConnectionError:
                            WifiManager.delete(ssid)
                            return False
                        return cell
                    else:
                        return False
                else:
                    scheme = WifiManager.add(cell)
                    try:
                        scheme.activate()
                    except wifi.exceptions.ConnectionError:
                        WifiManager.delete(ssid)
                        return False
                    return cell
        return False

    @staticmethod
    def add(cell, password=None):
        if not cell:
            return False
        scheme = wifi.Scheme.for_cell('wlan0', cell.ssid, cell, password)
        scheme.save()
        return scheme

    @staticmethod
    def delete(ssid):
        if not ssid:
            return False
        cell = WifiManager.find_from_saved_list(ssid)
        if cell:
            cell.delete()
            return True
        return False



class WifiAccessPointManager:
    @staticmethod
    def setup():
        WifiAccessPointManager.backup()
        os.system('mv /etc/dnsmasq.conf.original /etc/dnsmasq.conf')
        os.system('mv /etc/dhcpcd.conf.original /etc/dhcpcd.conf')
        os.system('reboot')

    @staticmethod
    def backup():
        os.system('mv /etc/wpa_supplicant/wpa_supplicant.conf /etc/wpa_supplicant/wpa_supplicant.conf.original')
        os.system('mv /etc/dnsmasq.conf /etc/dnsmasq.conf.original')
        os.system('mv /etc/dhcpcd.conf /etc/dhcpcd.conf.original')

    @staticmethod
    def restore():
        os.system('mv /etc/wpa_supplicant/wpa_supplicant.conf.original /etc/wpa_supplicant/wpa_supplicant.conf 2>/dev/null')
        os.system('mv /etc/dnsmasq.conf.original /etc/dnsmasq.conf 2>/dev/null')
        os.system('mv /etc/dhcpcd.conf.original /etc/dhcpcd.conf 2>/dev/null')
    
    @staticmethod
    def disable():
        os.system('cp /opt/raspberry-bot/src/configs/wifi/wpa_supplicant.conf.default /etc/wpa_supplicant/wpa_supplicant.conf')
        os.system('chmod 600 /etc/wpa_supplicant/wpa_supplicant.conf')
        os.system('rm /etc/dnsmasq.conf')
        os.system('rm /etc/hostapd/hostapd.conf')
        os.system('rm /etc/dhcpcd.conf')
        WifiAccessPointManager.restore()
    
    @staticmethod
    def is_access_point():
        iwconfig_out = subprocess.check_output(['iwconfig']).decode('utf-8')
        if "Mode:Master" in iwconfig_out:
            return True
        return False

    @staticmethod
    def watch_access_point_signal():
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        counter = 0
        # This is the main logic loop waiting for a button to be pressed on GPIO 18 for 10 seconds.
        # If that happens the device will reset to its AP Host mode allowing for reconfiguration on a new network.
        while True:
            while GPIO.input(18) == 1:
                time.sleep(1)
                counter = counter + 1
                if counter == 9:
                    print('Setting up the host as access point')
                    WifiAccessPointManager.setup()

                if GPIO.input(18) == 0:
                    counter = 0
                    break
            time.sleep(1)
