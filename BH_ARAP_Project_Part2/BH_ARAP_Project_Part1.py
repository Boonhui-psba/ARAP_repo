from controller import Robot, Camera

# -------------------------------
# Constants
# -------------------------------
TIME_STEP = 64
MAX_SPEED = 6.28
MULTIPLIER = 0.5

OBSTACLE_DISTANCE = 0.02

DOMINANCE_MARGIN = 40
MIN_INTENSITY = 120

# Horse detection settings
REGION_HALF_SIZE = 12
DETECTION_CONFIRM_FRAMES = 2

# -------------------------------
# Helper Functions
# -------------------------------
def get_distance_values(distance_sensors, distance_values):
    for i in range(8):
        val = distance_sensors[i].getValue() / 4096.0
        distance_values[i] = min(val, 1.0)


def front_obstacle(distance_values):
    avg = (distance_values[0] + distance_values[7]) / 2.0
    return avg > OBSTACLE_DISTANCE


def move_forward(left_motor, right_motor):
    left_motor.setVelocity(MAX_SPEED * MULTIPLIER)
    right_motor.setVelocity(MAX_SPEED * MULTIPLIER)


def move_backward(left_motor, right_motor, robot, timestep):
    left_motor.setVelocity(-MAX_SPEED * MULTIPLIER)
    right_motor.setVelocity(-MAX_SPEED * MULTIPLIER)
    wait(robot, timestep, 0.3)


def turn_left(left_motor, right_motor, robot, timestep):
    left_motor.setVelocity(-MAX_SPEED * MULTIPLIER)
    right_motor.setVelocity(MAX_SPEED * MULTIPLIER)
    wait(robot, timestep, 0.3)


def wait(robot, timestep, sec):
    start = robot.getTime()
    while robot.getTime() < start + sec:
        robot.step(timestep)


def get_camera_rgb(camera, interval, state):
    width = camera.getWidth()
    height = camera.getHeight()
    image = camera.getImage()

    if state["camera_interval"] >= interval:
        r = g = b = 0
        for x in range(width):
            for y in range(height):
                r += camera.imageGetRed(image, width, x, y)
                g += camera.imageGetGreen(image, width, x, y)
                b += camera.imageGetBlue(image, width, x, y)

        state["camera_interval"] = 0
        return (
            int(r / (width * height)),
            int(g / (width * height)),
            int(b / (width * height)),
        )
    else:
        state["camera_interval"] += 1
        return (0, 0, 0)


# -------------------------------
# Horse Detection (CLEAN)
# -------------------------------
def is_target_object(camera):
    width = camera.getWidth()
    height = camera.getHeight()
    image = camera.getImage()

    cx = width // 2
    cy = height // 2

    r = g = b = count = 0

    for x in range(cx - REGION_HALF_SIZE, cx + REGION_HALF_SIZE):
        for y in range(cy - REGION_HALF_SIZE, cy + REGION_HALF_SIZE):
            if 0 <= x < width and 0 <= y < height:
                r += camera.imageGetRed(image, width, x, y)
                g += camera.imageGetGreen(image, width, x, y)
                b += camera.imageGetBlue(image, width, x, y)
                count += 1

    if count == 0:
        return False

    r_avg = int(r / count)
    g_avg = int(g / count)
    b_avg = int(b / count)

    # Final tuned horse range
    return (
        136 <= r_avg <= 146 and
        122 <= g_avg <= 130 and
        108 <= b_avg <= 116
    )


def save_camera_image(camera):
    filename = "webots_horse.png"
    camera.saveImage(filename, 100)
    print(f"Image saved: {filename}")


# -------------------------------
# Main Robot Function
# -------------------------------
def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())

    # Distance sensors
    sensor_names = ("ps0","ps1","ps2","ps3","ps4","ps5","ps6","ps7")
    distance_sensors = []
    distance_values = [0.0] * 8

    for name in sensor_names:
        s = robot.getDevice(name)
        s.enable(timestep)
        distance_sensors.append(s)

    # Camera
    camera = robot.getDevice("camera")
    camera.enable(timestep)
    camera_state = {"camera_interval": 0}

    # Motors
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")

    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))

    encountered = set()
    captured = False
    detection_counter = 0

    # -------------------------------
    # Main Loop
    # -------------------------------
    while robot.step(timestep) != -1:

        get_distance_values(distance_sensors, distance_values)
        red, green, blue = get_camera_rgb(camera, 5, camera_state)

        # -------- PART 1: Colour Detection --------
        if not (red == 0 and green == 0 and blue == 0):
            detected_color = None

            if red > MIN_INTENSITY and red - max(green, blue) > DOMINANCE_MARGIN:
                detected_color = "Red"
            elif green > MIN_INTENSITY and green - max(red, blue) > DOMINANCE_MARGIN:
                detected_color = "Green"
            elif blue > MIN_INTENSITY and blue - max(red, green) > DOMINANCE_MARGIN:
                detected_color = "Blue"

            if detected_color and detected_color not in encountered:
                encountered.add(detected_color)
                print(f"I see {detected_color.lower()}")
                summary = ", ".join(sorted(encountered))
                print(f"Colours detected so far: {summary}")

        # -------- PART 2: Horse Detection --------
        if not captured:
            if is_target_object(camera):
                detection_counter += 1
                print(f"Horse detection counter: {detection_counter}")
            else:
                detection_counter = 0

            if detection_counter >= DETECTION_CONFIRM_FRAMES:
                print("Target horse detected")
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                wait(robot, timestep, 0.3)
                save_camera_image(camera)
                captured = True

        # -------- Movement --------
        if captured:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
        else:
            if front_obstacle(distance_values):
                move_backward(left_motor, right_motor, robot, timestep)
                turn_left(left_motor, right_motor, robot, timestep)
            else:
                move_forward(left_motor, right_motor)


# -------------------------------
# Entry Point
# -------------------------------
if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)