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
import math

# wifi signal class for each SSID
class signal:
    def __init__(self, parse, ssid):
        self.SSID = "Hidden"
        self.MAC = ""
        self.RSSI = 0
        self.dist = 0
        self.parseSignal(parse, ssid)

    # parse signal parameters to store SSID/MAC/RSSI/Dist
    def parseSignal(self, parse, ssid):
        for i in parse:
            if "BSSID" in i:
                self.MAC = i.split()[-1]
            if "Signal" in i:
                percent = i.split()[-1]
                self.RSSI = max(self.RSSI, float(percent[:-1]))
        self.getDist()

        name = ssid.split(":")[-1].strip()
        if name!="":
            self.SSID = name

    # convert RSSI to distance in m
    def getDist(self):
        fMhz = 2412     # frequency in MHz
        exp = (47.55 - 20*math.log10(fMhz) + abs(self.RSSI))/20
        self.dist = 10**exp

    # print details
    def print(self):
        print("SSID : ", self.SSID, " | MAC Address : ", self.MAC, " | RSSI : ", self.RSSI, " | Distance", self.dist)


# parse extracted data from command prompt
class WiFiScanner:
    def __init__(self):
        self.networks = []
        self.numNetworks = 0
        self.getSSIDRaw()

    def print(self):
        print(self.numNetworks," networks were discovered")
        for i in self.networks:
            i.print()

    def parseSSID(self, signals):
        idx = []

        # find all places where SSID is present
        for i in range(len(signals)):
            if "SSID" in signals[i] and "BSSID" not in signals[i]:
                idx.append(i)

        # parse the whole info per SSID
        prev = 0
        for next in idx:
            if next!=prev:
                self.networks.append(signal(signals[prev:next], signals[next]))
        self.networks.append(signal(signals[next:], signals[next]))

        # update number of networks
        self.numNetworks = len(self.networks)

    def getSSIDRaw(self):
        signals = subprocess.check_output(["netsh", "wlan", "show", "network", "mode=Bssid"])
        signals = signals.decode("ascii").replace("\r","").split("\n")[4:]
        signals[:] = [x for x in signals if x != ""]
        self.parseSSID(signals)
