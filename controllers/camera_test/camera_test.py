from controller import Robot, Camera

# ---------------------------------------------------------
# CAMERA TEST CONTROLLER
# PeraBots 2026 - Webots R2025a
# ---------------------------------------------------------

TIME_STEP = 32

# Create robot
robot = Robot()

# ---------------------------------------------------------
# GET CAMERA
# ---------------------------------------------------------

camera = robot.getDevice("front_camera")

if camera is None:
    print("[ERROR] Camera 'front_camera' was not found.")
    print("[ERROR] Check the Camera name in TwoWheelRobot.proto")
    robot.cleanup()
    exit()

# Enable camera
camera.enable(TIME_STEP)

# Camera information
width = camera.getWidth()
height = camera.getHeight()

print("========================================")
print("       CAMERA TEST STARTED")
print("========================================")
print(f"[CAMERA] Name       : front_camera")
print(f"[CAMERA] Resolution : {width} x {height}")
print(f"[CAMERA] FOV        : {camera.getFov():.3f} rad")
print("========================================")
print("[CAMERA] Robot will remain stationary.")
print("[CAMERA] Waiting for camera images...")
print("========================================")


# ---------------------------------------------------------
# MAIN LOOP
# ---------------------------------------------------------

step_count = 0

while robot.step(TIME_STEP) != -1:

    # Get camera image
    image = camera.getImage()

    if image is None:
        print("[CAMERA] No image received.")
        continue

    step_count += 1

    # Print confirmation every ~1 second
    if step_count % 30 == 0:

        # Get RGB value from center pixel
        center_x = width // 2
        center_y = height // 2

        r = camera.imageGetRed(
            image,
            width,
            center_x,
            center_y
        )

        g = camera.imageGetGreen(
            image,
            width,
            center_x,
            center_y
        )

        b = camera.imageGetBlue(
            image,
            width,
            center_x,
            center_y
        )

        print(
            f"[CAMERA] Image received | "
            f"Center RGB = ({r}, {g}, {b})"
        )