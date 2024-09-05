from controller import Robot
import math

def ankle_ik(d, L1, h1, h2, tx, ty):
    cx = math.cos(tx)
    sx = math.sin(tx)
    cy = math.cos(ty)
    sy = math.sin(ty)

    AL = -L1 * L1 * cy + L1 * d * sx * sy
    BL = -L1 * L1 * sy + L1 * h1 - L1 * d * sx * cy
    CL = -(L1 * L1 + d * d - d * d * cx - L1 * h1 * sy - d * h1 * sx * cy)
    LenL = math.sqrt(AL * AL + BL * BL)

    AR = -L1 * L1 * cy - L1 * d * sx * sy
    BR = -L1 * L1 * sy + L1 * h2 + L1 * d * sx * cy
    CR = -(L1 * L1 + d * d - d * d * cx - L1 * h2 * sy + d * h2 * sx * cy)
    LenR = math.sqrt(AR * AR + BR * BR)

    if LenL <= abs(CL) or LenR <= abs(CR):
        return (0.0, 0.0)
    else:
        tL_1 = math.asin(CL / LenL) - math.asin(AL / LenL)
        tL_2 = math.asin(CL / LenL) + math.acos(BL / LenL)

        tR_1 = math.asin(CR / LenR) - math.asin(AR / LenR)
        tR_2 = math.asin(CR / LenR) + math.acos(BR / LenR)

        assert abs(tL_1 - tL_2) < 1e-3, "tL_1 - tL_2 > 1e-3"
        assert abs(tR_1 - tR_2) < 1e-3, "tR_1 - tR_2 > 1e-3"

        return (tL_1, tR_1)

robot = Robot()
timestep = int(robot.getBasicTimeStep())

motor1 = robot.getDevice('ankle_l_motor')
motor2 = robot.getDevice('ankle_r_motor')

#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

count = 0
while robot.step(timestep) != -1:
        count += timestep
        des_pos = math.sin(count * 0.001 * 6) * 0.32
    des_pos_c = math.cos(count * 0.001 * 6) * 0.32

    # pos = ankle_ik(0.06, 0.12, 0.49, 0.37, des_pos, 0.0)
    # pos = ankle_ik(0.06, 0.12, 0.49, 0.37, 0.0, des_pos)
    # pos = ankle_ik(0.06, 0.12, 0.49, 0.37, des_pos, des_pos_c)
    pos = ankle_ik(0.06, 0.12, 0.49, 0.37, des_pos, des_pos_c)

    # motor1.setTorque(1)
    # motor2.setTorque(1)
    pos_roll = math.atan((0.06 / 0.06) * math.sin(0.5 * (-pos[1] + pos[0])))
    # print("avrage: %f" % (0.5 * (pos[0] + pos[1])))
    # print("actual: %f" % (des_pos_c))

    print("avrage: %f" % pos_roll)
    print("actual: %f" % (des_pos))

    motor1.setPosition(pos[0])
    motor2.setPosition(pos[1])

    # motor1.setPosition(0.5)
    # motor2.setPosition(0.5)