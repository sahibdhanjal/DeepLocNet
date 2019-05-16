'''
Refer the docs here:
1. https://stackoverflow.com/questions/2851233/how-can-i-retrieve-the-signal-strength-of-nearby-wireless-lan-networks-on-window
2. https://stackoverflow.com/questions/15797920/how-to-convert-wifi-signal-strength-from-quality-percent-to-rssi-dbm
3. https://stackoverflow.com/questions/18245849/how-to-transmit-android-real-time-sensor-data-to-computer?noredirect=1&lq=1
4. https://stackoverflow.com/questions/49186048/android-wifi-distance
5. https://stackoverflow.com/questions/11217674/how-to-calculate-distance-from-wifi-router-using-signal-strength
6. https://github.com/priyankark/PhonePi_SampleServer
7. https://github.com/ksasmit/Wireless-and-Mobile-networks-Fingerprint-based-localization-of-mobile-devices/blob/master/main.cpp
8. https://github.com/SamShue/RSSI-Localization-Simulator
9. https://github.com/t-lohani/Netwok-side-Localization
'''

#!/usr/bin/env python
import subprocess
import math, time
from pdb import set_trace as bp

class WiFiScanner:
    def __init__(self, ssids = None):
        self.nametoMAC = {}
        self.MACtoSignal = {}
        self.numNetworks = 0
        self.ssids = ssids
        self.update()

    def parseSignal(self, signal):
        vals = signal.split()
        name = ' '.join(x for x in vals[0:len(vals)-7])
        mac = vals[-7]
        chan = vals[-6]
        rssi = vals[-1]
        return name, mac, rssi

    def print(self):
        print("Printing SSIDS with MAC associations:  ")
        for i in self.nametoMAC:
            print(i, ": ", self.nametoMAC[i])
        
        print("\n\nPrinting MAC with rssi associations:  ")
        for i in self.MACtoSignal:
            print(i, ": ", self.MACtoSignal[i])

    def update(self):
        signals = subprocess.check_output(["nmcli", "-f", "SSID,BSSID,CHAN,FREQ,RATE,SIGNAL", "dev", "wifi"])
        signals = signals.decode("ascii").replace("\r","").split("\n")[4:]
        signals[:] = [x for x in signals if x != ""]
        
        if len(signals) == 0:
            print("Running 'nmcli dev wifi rescan' as NIC is inactive")
            subprocess.run(["nmcli", "dev", "wifi", "rescan"])
            time.sleep(2)

        for signal in signals:
            name, mac, rssi = self.parseSignal(signal)
            
            # add mac address association to name
            if name in self.nametoMAC: 
                if mac in self.nametoMAC[name]: pass
                else: self.nametoMAC[name].append(mac)

            else: self.nametoMAC[name] = [mac]
            
            # add rssi association to mac address
            if mac in self.MACtoSignal: self.MACtoSignal[mac].append(rssi)
            else: self.MACtoSignal[mac] = [rssi]


if __name__ == "__main__":
    ssids = ['Vulture Aviation', 'MGuest']
    scanner = WiFiScanner(ssids)
    scanner.print()
    