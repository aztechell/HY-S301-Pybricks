from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Direction
from pybricks.tools import wait, StopWatch
from pybricks.pupdevices import Motor
from pybricks.iodevices import PUPDevice

hub = PrimeHub()
stopwatch = StopWatch()

left_motor = Motor(Port.E, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.F)

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


# веса датчиков (центр = 0)
WEIGHTS = [-3, -2, -1, 0, 0, +1, +2, +3]


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