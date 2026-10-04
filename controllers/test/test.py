from controller import Robot

# --------------------------------------------------
# BASIC SETTINGS
# --------------------------------------------------

TIME_STEP = 32
TEST_SPEED = 2.0

# --------------------------------------------------
# CREATE ROBOT
# --------------------------------------------------

robot = Robot()

# --------------------------------------------------
# GET MOTORS
# --------------------------------------------------

left_motor = robot.getDevice("left_wheel_motor")
right_motor = robot.getDevice("right_wheel_motor")

# Continuous rotation
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

# --------------------------------------------------
# STOP INITIALLY
# --------------------------------------------------

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# --------------------------------------------------
# MAIN LOOP
# --------------------------------------------------

while robot.step(TIME_STEP) != -1:

    # Move both wheels at exactly the same speed
    left_motor.setVelocity(TEST_SPEED)
    right_motor.setVelocity(TEST_SPEED)