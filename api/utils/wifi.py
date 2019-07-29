import wifi
import subprocess


class WifiManager:
    @staticmethod
    def get_dict_of_ssids_with_status():
        wifis = WifiManager.scan()
        SSIDs = [wifi.ssid for wifi in wifis]
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
        return [wifi.Cell.all('wlan0')]

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