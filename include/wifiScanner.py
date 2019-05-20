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
10. https://stackoverflow.com/questions/15797920/how-to-convert-wifi-signal-strength-from-quality-percent-to-rssi-dbm
'''

#!/usr/bin/env python
import subprocess
import math, time, csv
from pdb import set_trace as bp

class WiFiScanner:
    def __init__(self, filename):
        self.file = filename
        self.TXName = []
        self.name2MAC = {}
        self.RSSI = {}
        self.numAPs = 0
        self.defineAPS()
        self.update()

    def signalStrength2RSSI(self, rssi):
        return max(-100, min(int(rssi)/2 - 100, -50))

    def parseSignal(self, signal):
        vals = signal.split()
        name = ' '.join(x for x in vals[0:len(vals)-7])
        mac = vals[-7]
        rssi = self.signalStrength2RSSI(vals[-1])
        return name, mac, rssi

    def update(self):
        signals = subprocess.check_output(["nmcli", "-f", "SSID,BSSID,CHAN,FREQ,RATE,SIGNAL", "dev", "wifi"])
        signals = signals.decode("ascii").replace("\r","").split("\n")[4:]
        signals[:] = [x for x in signals if x != ""]
        
        if len(signals) == 0:
            print("Running 'nmcli dev wifi rescan' as NIC is inactive")
            subprocess.run(["nmcli", "dev", "wifi", "rescan"])

        entries = [0]*self.numAPs

        for signal in signals:
            name, mac, rssi = self.parseSignal(signal)
            if name=="MGuest":
                mac = mac.lower()
                if mac in self.MAC2Name.keys():
                    ID = self.MAC2Name[mac]
                    self.RSSI[ID] = rssi
                    idx = self.TXName.index(ID)
                    entries[idx] = rssi
        
        val = ", ".join(str(x) for x in entries) + '\n'
        trial = open(self.file, 'a')
        trial.write(val)

    def defineAPS(self):
        # 45 APs defined
        self.TXName = [ '106', '10M', '111', '114-H', '115', '116', '117', '120', '121', '124', '125', '125-H', '129', '130', 
                        '131', '134-E', '134-W', '135', '137', '138-E', '138-W', '139', '203', '203-H', '204', '208', '209', 
                        '210', '211', '212', '213', '214', '216', '219', '220', '221', '222', '229', '232', '233', '235', 
                        '236', '242', '244-E', '244-W' ]
        self.numAPs = len(self.TXName)

        self.name2MAC = {'106'   : '2c:33:11:89:6d:c2' , '10M'   : '2c:33:11:68:49:92' , '111'   : '2c:33:11:89:68:82' , '114-H' : '2c:33:11:3c:53:12' , '115'   : '2c:33:11:9e:06:32' , 
                        '116'   : '2c:33:11:3c:53:62' , '117'   : '2c:33:11:5b:a1:92' , '120'   : 'e8:65:49:c2:d4:62' , '121'   : '2c:33:11:5b:a5:e2' , '124'   : '2c:33:11:89:42:f2' ,
                        '125'   : '2c:33:11:3c:55:12' , '125-H' : '2c:33:11:89:6a:52' , '129'   : '38:90:a5:19:87:e2' , '130'   : '2c:33:11:68:48:82' , '131'   : '2c:33:11:9e:0a:32' ,
                        '134-E' : '2c:33:11:9e:07:f2' , '134-W' : '2c:33:11:9e:0d:82' , '135'   : '2c:33:11:5b:a6:e2' , '137'   : '2c:33:11:9e:05:42' , '138-E' : '2c:33:11:9e:0a:92' ,
                        '138-W' : '2c:33:11:5b:a5:32' , '139'   : '2c:33:11:89:48:22' , '203'   : '2c:33:11:3c:52:a2' , '203-H' : '2c:33:11:5b:95:52' , '204'   : '2c:33:11:89:69:72' ,
                        '208'   : '2c:33:11:49:49:c2' , '209'   : '2c:33:11:89:63:72' , '210'   : '2c:33:11:89:6e:12' , '211'   : '2c:33:11:89:61:62' , '212'   : '2c:33:11:5b:9e:82' ,
                        '213'   : '2c:33:11:5b:9d:82' , '214'   : '2c:33:11:68:2e:32' , '216'   : '2c:33:11:68:43:52' , '219'   : '2c:33:11:9e:03:12' , '220'   : '2c:33:11:5b:a4:52' ,
                        '221'   : '2c:33:11:21:b6:22' , '222'   : '2c:33:11:9e:02:d2' , '229'   : '2c:33:11:5b:a3:72' , '232'   : '2c:33:11:68:2e:62' , '233'   : '2c:33:11:68:43:b2' ,
                        '235'   : '2c:33:11:89:63:a2' , '236'   : '2c:33:11:3c:40:a2' , '242'   : '2c:33:11:89:54:12' , '244-E' : '2c:33:11:89:69:22' , '244-W' : '2c:33:11:5b:a4:42' }

        self.MAC2Name = {'2c:33:11:89:6d:c2' : '106' ,  '2c:33:11:68:49:92' : '10M' ,  '2c:33:11:89:68:82' : '111' ,  '2c:33:11:3c:53:12' : '114-H' ,  '2c:33:11:9e:06:32' : '115' ,
                        '2c:33:11:3c:53:62' : '116' ,  '2c:33:11:5b:a1:92' : '117' ,  'e8:65:49:c2:d4:62' : '120' ,  '2c:33:11:5b:a5:e2' : '121' ,  '2c:33:11:89:42:f2' : '124' ,
                        '2c:33:11:3c:55:12' : '125' ,  '2c:33:11:89:6a:52' : '125-H' ,  '38:90:a5:19:87:e2' : '129' ,  '2c:33:11:68:48:82' : '130' ,  '2c:33:11:9e:0a:32' : '131' ,
                        '2c:33:11:9e:07:f2' : '134-E' ,  '2c:33:11:9e:0d:82' : '134-W' ,  '2c:33:11:5b:a6:e2' : '135' ,  '2c:33:11:9e:05:42' : '137' ,  '2c:33:11:9e:0a:92' : '138-E' ,
                        '2c:33:11:5b:a5:32' : '138-W' ,  '2c:33:11:89:48:22' : '139' ,  '2c:33:11:3c:52:a2' : '203' ,  '2c:33:11:5b:95:52' : '203-H' ,  '2c:33:11:89:69:72' : '204' ,
                        '2c:33:11:49:49:c2' : '208' ,  '2c:33:11:89:63:72' : '209' ,  '2c:33:11:89:6e:12' : '210' ,  '2c:33:11:89:61:62' : '211' ,  '2c:33:11:5b:9e:82' : '212' ,
                        '2c:33:11:5b:9d:82' : '213' ,  '2c:33:11:68:2e:32' : '214' ,  '2c:33:11:68:43:52' : '216' ,  '2c:33:11:9e:03:12' : '219' ,  '2c:33:11:5b:a4:52' : '220' ,
                        '2c:33:11:21:b6:22' : '221' ,  '2c:33:11:9e:02:d2' : '222' ,  '2c:33:11:5b:a3:72' : '229' ,  '2c:33:11:68:2e:62' : '232' ,  '2c:33:11:68:43:b2' : '233' ,
                        '2c:33:11:89:63:a2' : '235' ,  '2c:33:11:3c:40:a2' : '236' ,  '2c:33:11:89:54:12' : '242' ,  '2c:33:11:89:69:22' : '244-E' ,  '2c:33:11:5b:a4:42' : '244-W' }                       

if __name__ == "__main__":
    scanner = WiFiScanner("trial.csv")
    print(scanner.RSSI)