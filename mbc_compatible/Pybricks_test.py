from pybricks.hubs import PrimeHub
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.iodevices import PUPDevice

hub = PrimeHub()

lineSensorDevice = PUPDevice(Port.A)

def lineSensorDataUpdate():
    """Обновляет данные из датчика."""
    while True:
        try:
            data = lineSensorDevice.read(9)
        except OSError:
            print("OS")
            wait(2)
            continue  # нет данных → пробуем снова

        if len(data) != 7:
            print("len", len(data))
            wait(1)
            continue  # пакет не полный → снова пробуем

        # распаковка 7 слов
        data1, data2, data3, data4, data5, data6, data7 = data
        
        lineSensor = [100] * 8
        lineSensor[0] = data1 & 0xFF
        lineSensor[1] = (data1 >> 8) & 0xFF
        lineSensor[2] = data2 & 0xFF
        lineSensor[3] = (data2 >> 8) & 0xFF
        lineSensor[4] = data3 & 0xFF
        lineSensor[5] = (data3 >> 8) & 0xFF
        lineSensor[6] = data4 & 0xFF
        lineSensor[7] = (data4 >> 8) & 0xFF

        lineBlackError = (data5 & 0xFF) - 8
        lineBlackWidth = (data5 >> 8) & 0xFF
        lineBlackBin   = data6 & 0xFF
        lineWhiteError = ((data6 >> 8) & 0xFF) - 8
        lineWhiteWidth = data7 & 0xFF
        lineWhiteBin   = (data7 >> 8) & 0xFF

        break
    return lineSensor
    # если надо вернуть всё → раскомментируй:
    # return lineSensor, lineBlackError, lineBlackWidth, lineBlackBin, lineWhiteError, lineWhiteWidth, lineWhiteBin


while True:
    print(lineSensorDataUpdate())
    wait(100)
