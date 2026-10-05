from controller import Robot

robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

imu = robot.getDevice("imu")
imu.enable(TIME_STEP)

print("IMU test")

while robot.step(TIME_STEP) != -1:

    roll, pitch, yaw = imu.getRollPitchYaw()

    print(
        f"Roll={roll:.3f} | "
        f"Pitch={pitch:.3f} | "
        f"Yaw={yaw:.3f}"
    )