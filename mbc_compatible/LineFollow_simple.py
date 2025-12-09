from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import PUPDevice

hub = PrimeHub()
stopwatch = StopWatch()

left_motor = Motor(Port.E, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.F)

lineSensorDevice = PUPDevice(Port.A)


def lineSensorDataUpdate():
    """Чтение 7×DATA16 пакета из режима CALIB."""
    while True:
        try:
            data = lineSensorDevice.read(9)
        except OSError:
            print("OS")
            wait(2)
            continue

        if len(data) != 7:
            print("len", len(data))
            wait(1)
            continue

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

        # Если понадобятся доп. параметры — можно вернуть и их.
        break

    return lineSensor


def line_error():
    lineSensor = lineSensorDataUpdate()
    sumL = sum(lineSensor[0:4])
    sumR = -sum(lineSensor[4:8])
    return sumL + sumR


def line_follow_dist_PD(Kp, Kd, base_speed, dist):

    target_angle_rotations = (dist / (3.14159 * 44)) * 360

    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    dt = 0.005
    prev_e = line_error()

    while (abs(left_motor.angle()) + abs(right_motor.angle())) / 2 <= target_angle_rotations:

        t0 = stopwatch.time()

        e = line_error()
        turn = Kp * e + Kd * (e - prev_e) / dt

        left_motor.run(base_speed + turn)
        right_motor.run(base_speed - turn)

        prev_e = e

        elapsed = (stopwatch.time() - t0) / 1000.0
        wait(int(max(0.0, (dt - elapsed)) * 1000))

    left_motor.stop()
    right_motor.stop()


line_follow_dist_PD(3, 1, 1000, 1000)
wait(100)
