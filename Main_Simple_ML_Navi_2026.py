import serial
import time
import sys
import cv2
import numpy as np
import os

# Import Edge Impulse Runner
from edge_impulse_linux.image import ImageImpulseRunner

# ====== Constants & Paths ======
NAV_SERIAL = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_8513332393535170A1E0-if00"
BAUD_RATE = 115200

MODEL_FILENAME = 'bee3900_25.eim'
IMAGE_SAVE_DIR = '/home/pi/Desktop/Image'

# ====== Global Variables ======
arduino_nav = None
runner = None
video = None

def shutdown_routine():
    if arduino_nav:
        arduino_nav.write(b"!\n") # Send emergency halt
        arduino_nav.close()
    if runner:
        runner.stop()
    if video:
        video.release()
    cv2.destroyAllWindows()

# ====== Serial Functions (from Main_Simple_Navi_2026.py) ======
def connect_serial():
    global arduino_nav
    print("Connecting to Arduino...")
    try:
        arduino_nav = serial.Serial(NAV_SERIAL, BAUD_RATE, timeout=1)
        time.sleep(2)
        print("Connected successfully to Arduino!\n")
    except Exception as e:
        print(f"SERIAL FAILED! {e}")
        time.sleep(3)
        shutdown_routine()
        sys.exit(1)

def send_nav(cmd, delay=0.2, wait_for_done=True):
    try:
        arduino_nav.write(f"{cmd}\n".encode())
        print(f"  -> Sent: {cmd}")
        if wait_for_done:
            start_time = time.time()
            while time.time() - start_time < 20.0:
                if arduino_nav.in_waiting:
                    line = arduino_nav.readline().decode('utf-8').strip()
                    if line.startswith("MSG:"):
                        pass
                    elif line == "DONE":
                        break
                    elif line == "HALTED":
                        print("  [Arduino Acknowledged Halt]")
                        sys.exit(0)
        time.sleep(delay)
    except serial.SerialException:
        print("Serial communication error.")

# ====== Vision Functions (from Main_Simple_ML_2026.py) ======
def init_vision():
    global runner, video

    if not os.path.exists(IMAGE_SAVE_DIR):
        os.makedirs(IMAGE_SAVE_DIR)

    print("Initializing Edge Impulse model...")
    try:
        dir_path = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(dir_path, MODEL_FILENAME)
        runner = ImageImpulseRunner(model_path)
        runner.init()
    except Exception as e:
        print("AI MODEL FAILED!", str(e))
        sys.exit(1)

    print("Warming up webcam...")
    video = cv2.VideoCapture(0)
    video.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    time.sleep(1) # Let the camera sensor warm up

def identify_plant():
    # 1. Flush the camera buffer to get a fresh, real-time frame
    for _ in range(5):
        video.grab()

    ret, raw_frame = video.read()
    if not ret:
        print("Error: Failed to grab frame from camera.")
        return "EMPTY"

    # 2. Prevent Side Cropping (Square Padding)
    h, w = raw_frame.shape[:2]
    diff = w - h
    top, bottom = diff // 2, diff // 2
    square_frame = cv2.copyMakeBorder(raw_frame, top, bottom, 0, 0, cv2.BORDER_CONSTANT, value=[0, 0, 0])
    rgb_frame = cv2.cvtColor(square_frame, cv2.COLOR_BGR2RGB)

    # 3. Run Inference
    features, cropped = runner.get_features_from_image(rgb_frame)
    res = runner.classify(features)

    # 4. Process Bounding Boxes
    valid_boxes = []
    if "bounding_boxes" in res["result"]:
        valid_boxes = [b for b in res["result"]["bounding_boxes"] if b['value'] > 0.5]

    num_plants = len(valid_boxes)

    # 5. Determine Plant Type
    if num_plants == 0:
        plant_type = "EMPTY"
    elif num_plants == 1:
        plant_type = "SINGLE"
    else:
        plant_type = "DOUBLE"

    # 6. Save Raw Image for Debugging
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    img_filename = os.path.join(IMAGE_SAVE_DIR, f"plant_{timestamp}_{plant_type}.jpg")
    cv2.imwrite(img_filename, raw_frame)

    # 7. Draw Boxes on Camera View
    cropped_h, cropped_w = cropped.shape[:2]
    ratio_x = square_frame.shape[1] / cropped_w
    ratio_y = square_frame.shape[0] / cropped_h
    display_frame = square_frame.copy()

    for b in valid_boxes[:2]:
        x = int(b['x'] * ratio_x)
        y = int(b['y'] * ratio_y)
        bw = int(b['width'] * ratio_x)
        bh = int(b['height'] * ratio_y)
        cx = x + bw // 2
        cy = y + bh // 2
        label_text = b['label']

        box_color = (0, 255, 0) if label_text.lower() == "good" else (255, 0, 255)
        cv2.circle(display_frame, (cx, cy), 5, box_color, -1)
        cv2.rectangle(display_frame, (x, y), (x + bw, y + bh), box_color, 2)
        cv2.putText(display_frame, f"{label_text} ({b['value']:.2f})", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

    cv2.imshow("Robot AI Vision", display_frame)
    cv2.waitKey(1)
    return plant_type

# ====== Main Routine: See a plant, then move ======
def run_vision_move():
    init_vision()
    connect_serial()
    print("\nStarting Vision + Move Test.")
    print("SINGLE -> F100 (forward 100 mm), DOUBLE -> B100 (backward 100 mm), EMPTY -> stay.")
    print("Press 'q' while clicked on the video window or Ctrl+C in terminal to exit.\n")

    try:
        while True:
            # Run the plant identification function
            plant_type = identify_plant()
            print(f"Detected Plant Status: {plant_type}")

            # Move the robot based on what the camera saw
            if plant_type == "SINGLE":
                send_nav("F100")    # Move forward 100 mm, wait for DONE
            elif plant_type == "DOUBLE":
                send_nav("B100")    # Move backward 100 mm, wait for DONE
            # EMPTY: no movement

            # Pause for 1 second before looking again, and check if 'q' was pressed
            if cv2.waitKey(1000) & 0xFF == ord('q'):
                print("User pressed 'q'. Exiting...")
                break

    except KeyboardInterrupt:
        print("\n[STOP] Keyboard interrupt detected.")
    finally:
        shutdown_routine()  # Stops the motors ("!"), closes the port, camera and model

if __name__ == "__main__":
    run_vision_move()
