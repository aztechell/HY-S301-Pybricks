from pybricks.hubs import PrimeHub
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.iodevices import PUPDevice

hub = PrimeHub()
lineSensorDevice = PUPDevice(Port.A)

def lineSensorDataUpdate():
    while True:
        try:
            data = lineSensorDevice.read(0)
        except OSError:
            wait(2000)
            continue
        if len(data) != 8:
            wait(1000)
            continue

        return [(v & 0xFF) for v in data]

while True:
    print(lineSensorDataUpdate())
    wait(100)
