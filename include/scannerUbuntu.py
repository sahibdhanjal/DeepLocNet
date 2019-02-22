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
    scanner.update()
    scanner.print()
    