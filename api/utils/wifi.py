import wifi
import subprocess


class WifiManager:
    @staticmethod
    def get_dict_of_ssids_with_status():
        SSIDs = WifiManager.scan()
        current_ssid = WifiManager.get_currently_connected_ssid()
        ssid_dict = {}
        for ssid in SSIDs:
            if ssid == current_ssid:
                ssid_dict[ssid] = 'connected'
            else:
                ssid_dict[ssid] = 'disconnect'
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
    def setup():
        os.system('mkdir /etc/raspberry-bot')
        os.system('rm -f ./tmp/*')
        os.system('cp /opt/raspberry-bot/src/api/libs/wifi/static_files/dnsmasq.conf /etc/')
        os.system('cp /opt/raspberry-bot/src/api/libs/wifi/static_files/hostapd.conf.wpa /etc/hostapd/hostapd.conf')
        os.system('cp /opt/raspberry-bot/src/api/libs/wifi/static_files/dhcpcd.conf /etc/')
        os.system('mkdir /etc/cron.raspberrybot-wifi')
        os.system('cp /opt/raspberry-bot/src/api/libs/wifi/static_files/aphost_bootstrapper /etc/cron.raspberrybot-wifi')
        os.system('chmod +x /etc/cron.raspberrybot-wifi/aphost_bootstrapper')
        os.system('echo "# RaspberryBot Wifi Access Point Manager Startup" >> /etc/crontab')
        os.system('echo "@reboot root run-parts /etc/cron.raspberrybot-wifi/" >> /etc/crontab')
        os.system('mv /opt/raspberry-bot/src/api/libs/wifi/static_files/raspberrybot-wifi.conf /etc/raspberry-bot')
        os.system('touch /etc/raspberry-bot/host_mode')

    
    def backup():
        os.system('mv /etc/wpa_supplicant/wpa_supplicant.conf /etc/wpa_supplicant/wpa_supplicant.conf.original')
        os.system('mv /etc/dnsmasq.conf /etc/dnsmasq.conf.original')
        os.system('mv /etc/dhcpcd.conf /etc/dhcpcd.conf.original')

    def restore():
        os.system('mv /etc/wpa_supplicant/wpa_supplicant.conf.original /etc/wpa_supplicant/wpa_supplicant.conf')
        os.system('mv /etc/dnsmasq.conf.original /etc/dnsmasq.conf')
        os.system('mv /etc/dhcpcd.conf.original /etc/dhcpcd.conf')
    
    def disable():
        os.system('cp /opt/raspberry-bot/src/api/libs/wifi/static_files/wpa_supplicant.conf.default /etc/wpa_supplicant/wpa_supplicant.conf')
        os.system('chmod 600 /etc/wpa_supplicant/wpa_supplicant.conf')
        os.system('mv /etc/wpa_supplicant/wpa_supplicant.conf.original /etc/wpa_supplicant/wpa_supplicant.conf 2>/dev/null')
        os.system('rm -rf /etc/cron.raspberrybot-wifi')
        os.system('rm /etc/dnsmasq.conf')
        os.system('mv /etc/dnsmasq.conf.original /etc/dnsmasq.conf 2>/dev/null')
        os.system('rm /etc/hostapd/hostapd.conf')
        os.system('rm /etc/dhcpcd.conf')
        os.system('mv /etc/dhcpcd.conf.original /etc/dhcpcd.conf 2>/dev/null')
        os.system('sed -i \'s/# RaspberryBot Wifi Access Point Manager Startup//\' /etc/crontab')
        os.system('sed -i \'s/@reboot root run-parts \/etc\/cron.raspberrybot-wifi\///\' /etc/crontab')


