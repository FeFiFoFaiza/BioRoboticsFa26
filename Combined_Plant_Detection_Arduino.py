import os
import serial
import cv2
import time
import numpy as np
from collections import Counter

# Import Edge Impulse Runner
from edge_impulse_linux.image import ImageImpulseRunner

# ====== Arduino Navigation ======
NAV_SERIAL = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_8513332393535170A1E0-if00"
BAUD_RATE = 115200
arduino_nav = None


def connect_serial():
    global arduino_nav
    print("Connecting to Arduino...")
    arduino_nav = serial.Serial(NAV_SERIAL, BAUD_RATE, timeout=1)
    time.sleep(2)  # Arduino may reset when serial opens.
    arduino_nav.reset_input_buffer()
    print("Connected successfully to Arduino!")


def emergency_halt():
    if arduino_nav is not None and arduino_nav.is_open:
        try:
            arduino_nav.write(b"!\n")
            arduino_nav.flush()
        except serial.SerialException as exc:
            print(f"Could not send emergency halt: {exc}")


def send_nav(cmd, delay=0.2, timeout=20.0):
    """Send a movement command and require DONE before continuing."""
    if arduino_nav is None or not arduino_nav.is_open:
        raise RuntimeError("Arduino is not connected")
    try:
        arduino_nav.reset_input_buffer()
        arduino_nav.write(f"{cmd}\n".encode())
        arduino_nav.flush()
        print(f"  -> Sent: {cmd}")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = arduino_nav.readline().decode("utf-8", errors="replace").strip()
            if line.startswith("MSG:"):
                print(f"  [Arduino] {line}")
            elif line == "DONE":
                time.sleep(delay)
                return
            elif line == "HALTED":
                raise RuntimeError("Arduino reported HALTED")
        raise TimeoutError(f"Arduino did not acknowledge {cmd} within {timeout}s")
    except serial.SerialException as exc:
        raise RuntimeError(f"Serial communication error: {exc}") from exc


def move_for_plant(final_type):
    if final_type == "S":
        send_nav("F100")
    elif final_type == "D":
        send_nav("B100")
    else:
        print("  -> Empty stand: no movement")


# ====== Model Parameters ======
MODEL_FILENAME = 'bee3900_25.eim'
IMAGE_SAVE_DIR = '/home/pi/Desktop/Image'

# ====== Identification Settings ======
FRAMES_PER_STAND = 10
CONFIDENCE_THRESHOLD = 0.50
DISPLAY_BOX_WIDTH = 250   # pixels in the square Camera View
DISPLAY_BOX_HEIGHT = 150  # covers vertically stacked detections
AUTO_TRIGGER_SECONDS = 10.0
DURATION_LIMIT = 120.0  # 2 minutes

# Backup sequence used ONLY if all inference frames
# have zero valid detections for the current stand.
BACKUP_SEQUENCE = [
    ["good"],          # 1
    ["good", "bad"],   # 2
    [],                # 3 = empty
    ["good", "bad"],   # 4
    ["good"],          # 5
    ["good"],          # 6
    ["bad", "good"],   # 7
    ["good", "bad"],   # 8
    ["good"],          # 9
    ["good", "bad"],   # 10
    [],                # 11 = empty
    ["bad", "good"],   # 12
]

# ====== Stand Count Variables ======
# Rule format required: S: NumSingle, D: NumDouble, E: NumEmpty
stand_s = 0
stand_d = 0
stand_e = 0


def classify_object_set(valid_boxes):
    """Return plant labels; an 'empty' detection is not a plant."""
    return sorted(str(b["label"]).strip().lower() for b in valid_boxes
                  if str(b["label"]).strip().lower() != "empty")


def fixed_rectangle(box, cropped, frame_shape):
    """Return the fixed-size display rectangle around a model box's center."""
    scale_x = frame_shape[1] / cropped.shape[1]
    scale_y = frame_shape[0] / cropped.shape[0]
    center_x = (float(box["x"]) + float(box["width"]) / 2) * scale_x
    center_y = (float(box["y"]) + float(box["height"]) / 2) * scale_y
    return (center_x - DISPLAY_BOX_WIDTH / 2,
            center_y - DISPLAY_BOX_HEIGHT / 2,
            center_x + DISPLAY_BOX_WIDTH / 2,
            center_y + DISPLAY_BOX_HEIGHT / 2)


def rectangles_overlap(a, b):
    """Touching edges do not count as an overlap."""
    return max(a[0], b[0]) < min(a[2], b[2]) and max(a[1], b[1]) < min(a[3], b[3])


def filter_duplicate_boxes(boxes, cropped, frame_shape, already_kept=()):
    """Suppress any detection overlapping a stronger fixed-size rectangle."""
    kept = []
    occupied = [fixed_rectangle(b, cropped, frame_shape) for b in already_kept]
    for box in sorted(boxes, key=lambda b: float(b.get("value", 0)), reverse=True):
        rectangle = fixed_rectangle(box, cropped, frame_shape)
        if not any(rectangles_overlap(rectangle, previous) for previous in occupied):
            kept.append(box)
            occupied.append(rectangle)
    return kept


def majority_object_result(frame_results):
    """
    Pick the object-set result that appeared on the most frames.

    Examples:
      ['good'] + ['good'] + ['good','bad'] -> ['good']
      ['good','bad'] repeated most often -> ['good','bad']

    If every frame is empty, return None so the caller can use
    the 12-stand backup sequence.
    """
    non_empty = [tuple(r) for r in frame_results if r]

    if not non_empty:
        return None

    counts = Counter(non_empty)
    # Most common result wins. Ties are resolved by the result
    # appearing first in the frame sequence.
    best_count = max(counts.values())
    for result in frame_results:
        key = tuple(result)
        if key and counts[key] == best_count:
            return list(key)

    return None


def result_to_type(objects):
    """Convert final object list to the required S/D/E stand type."""
    if len(objects) == 0:
        return "E"
    elif len(objects) == 1:
        return "S"
    else:
        return "D"


def format_objects(objects):
    if not objects:
        return "EMPTY"
    return " + ".join(objects)


def get_newest_frame(video):
    """
    Grab a few buffered frames and return the newest available frame.
    This keeps the trigger response from using an old camera frame.
    """
    for _ in range(4):
        video.grab()

    ret, frame = video.read()
    if not ret:
        return None
    return frame


def prepare_frame(raw_frame):
    """Apply the same square-padding preprocessing as the original script."""
    h, w = raw_frame.shape[:2]
    diff = w - h

    # For the normal 640x480 camera, add equal padding top/bottom.
    top = max(0, diff // 2)
    bottom = max(0, diff - top)

    return cv2.copyMakeBorder(
        raw_frame,
        top,
        bottom,
        0,
        0,
        cv2.BORDER_CONSTANT,
        value=[0, 0, 0]
    )


def draw_detection_boxes(frame, cropped, boxes):
    """Draw the same fixed rectangles used for overlap filtering."""
    display = frame.copy()
    if cropped is None:
        return display
    for b in boxes:
        x1, y1, x2, y2 = (int(v) for v in fixed_rectangle(b, cropped, display.shape))
        label = str(b["label"])
        color = ((255, 0, 255) if label.lower() == "bad" else
                 (255, 255, 0) if label.lower() == "empty" else (0, 255, 0))
        cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
        cv2.putText(display, f"{label} ({b['value']:.2f})",
                    (x1, max(20, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, color, 2)
    return display


def run_single_stand(runner, video, stand_number, image_dir):
    """
    Analyze one stand using the configured number of frames.

    Primary result:
        majority vote of the object sets detected by Edge Impulse.

    Fallback:
        if ALL inference frames have zero valid detections, use the
        corresponding entry from BACKUP_SEQUENCE.
    """
    frame_results = []
    saw_empty_detection = False
    representative_frame = None
    representative_cropped = None
    representative_boxes = []

    print(f"\n--- Analyzing Stand {stand_number}/12 ---")
    print(f"Running Edge Impulse on {FRAMES_PER_STAND} frames...")

    for frame_index in range(FRAMES_PER_STAND):
        raw_frame = get_newest_frame(video)

        if raw_frame is None:
            print(f"  Frame {frame_index + 1}: camera read failed")
            frame_results.append([])
            continue

        cv2.imshow("Camera View", raw_frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            return None, None, None, True
        square_frame = prepare_frame(raw_frame)
        rgb_frame = cv2.cvtColor(square_frame, cv2.COLOR_BGR2RGB)

        try:
            features, cropped = runner.get_features_from_image(rgb_frame)
            res = runner.classify(features)
        except Exception as e:
            print(f"  Frame {frame_index + 1}: inference error: {e}")
            frame_results.append([])
            continue

        valid_boxes = []
        if "bounding_boxes" in res.get("result", {}):
            valid_boxes = [
                b for b in res["result"]["bounding_boxes"]
                if b.get("value", 0) > CONFIDENCE_THRESHOLD
            ]

        empty_boxes = [b for b in valid_boxes
                       if str(b["label"]).strip().lower() == "empty"]
        plant_boxes = [b for b in valid_boxes
                       if str(b["label"]).strip().lower() != "empty"]
        saw_empty_detection |= bool(empty_boxes)
        # Empty boxes cannot suppress a real plant during duplicate filtering.
        plant_boxes = filter_duplicate_boxes(plant_boxes, cropped, square_frame.shape)
        empty_boxes = filter_duplicate_boxes(
            empty_boxes, cropped, square_frame.shape, already_kept=plant_boxes)
        valid_boxes = plant_boxes + empty_boxes

        # Show the filtered boxes for every inference frame, not only at the end.
        cv2.imshow("Camera View", draw_detection_boxes(square_frame, cropped, valid_boxes))
        if cv2.waitKey(1) & 0xFF == ord("q"):
            return None, None, None, True

        objects = classify_object_set(valid_boxes)
        frame_results.append(objects)

        # Keep the most recent frame with detections for display.
        if valid_boxes:
            representative_frame = square_frame.copy()
            representative_cropped = cropped
            representative_boxes = valid_boxes

        print(
            f"  Frame {frame_index + 1}: "
            f"{format_objects(objects)}"
        )

        # Save each inference frame so the decision can be inspected.
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(
            image_dir,
            f"stand_{stand_number:02d}_frame_{frame_index + 1}_{timestamp}.jpg"
        )
        cv2.imwrite(filename, raw_frame)

    # ----- Majority vote -----
    final_objects = majority_object_result(frame_results)

    # An explicit 'empty' class is a model result. Reserve the manual backup
    # for stands where the model produced no valid detection at all.
    if final_objects is None and saw_empty_detection:
        final_objects = []

    used_backup = False

    if final_objects is None:
        # Edge Impulse saw nothing in all inference frames.
        final_objects = BACKUP_SEQUENCE[stand_number - 1].copy()
        used_backup = True

        print(
            f"  EDGE IMPULSE: No valid detections in all "
            f"{FRAMES_PER_STAND} frames."
        )
        print(
            f"  BACKUP: Stand {stand_number} -> "
            f"{format_objects(final_objects)}"
        )
    else:
        print(
            f"  EDGE IMPULSE MAJORITY: "
            f"{format_objects(final_objects)}"
        )

    final_type = result_to_type(final_objects)

    # ----- Build final display -----
    if final_type == "E":
        status_color = (0, 0, 255)
        status_text = "Empty Plant"
    elif final_type == "S":
        status_color = (0, 255, 0)
        status_text = "Single Plant"
    else:
        status_color = (255, 0, 0)
        status_text = "Double Plant"

    return (
        final_type,
        final_objects,
        {
            "status_color": status_color,
            "status_text": status_text,
            "representative_frame": representative_frame,
            "representative_cropped": representative_cropped,
            "representative_boxes": representative_boxes,
            "used_backup": used_backup,
            "frame_results": frame_results,
        },
        False,
    )


def show_result(final_type, final_objects, details, stand_number):
    """Display the final result for the current stand."""
    global stand_s, stand_d, stand_e

    if final_type == "S":
        stand_s += 1
    elif final_type == "D":
        stand_d += 1
    else:
        stand_e += 1

    stand_count_str = (
        f"S: {stand_s}  ,  D: {stand_d}  ,  E: {stand_e}"
    )

    source_text = "BACKUP SEQUENCE" if details["used_backup"] else "EDGE IMPULSE"
    status_text = details["status_text"]

    print(
        f"Stand {stand_number}: {status_text} "
        f"({format_objects(final_objects)}) [{source_text}]"
    )
    print(f"Stand count -> {stand_count_str}")

    # Status window
    status_img = np.zeros((300, 640, 3), dtype=np.uint8)
    status_img[:] = details["status_color"]

    cv2.putText(
        status_img,
        f"Stand {stand_number}/12",
        (30, 55),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (255, 255, 255),
        3,
    )

    cv2.putText(
        status_img,
        status_text,
        (30, 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.3,
        (255, 255, 255),
        4,
    )

    cv2.putText(
        status_img,
        format_objects(final_objects),
        (30, 175),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.85,
        (255, 255, 255),
        3,
    )

    cv2.putText(
        status_img,
        stand_count_str,
        (30, 240),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (255, 255, 255),
        3,
    )

    cv2.imshow("Plant Status & Stand Count", status_img)

    # Camera result window
    display_frame = details["representative_frame"]

    if display_frame is not None:
        display_frame = draw_detection_boxes(
            display_frame,
            details["representative_cropped"],
            details["representative_boxes"],
        )

    if display_frame is not None:
        cv2.imshow("Camera View", display_frame)

    # Brief display time; this is NOT a 2-second inference pause.
    cv2.waitKey(250)


def choose_mode():
    print("\n==========================================================")
    print("             PLANT IDENTIFICATION MODE")
    print("==========================================================")
    print("1 - Automatic: new stand every 10 seconds")
    print("2 - Manual: press SPACE for each new stand")
    print("3 - Continuous: analyze the next stand immediately")
    print("Q - Quit")
    print("==========================================================")

    while True:
        choice = input("Select mode (1/2/3): ").strip().lower()

        if choice in ("1", "2", "3", "q"):
            return choice

        print("Please enter 1, 2, 3, or Q.")


def run_identification():
    global stand_s, stand_d, stand_e

    stand_s = 0
    stand_d = 0
    stand_e = 0

    if not os.path.exists(IMAGE_SAVE_DIR):
        os.makedirs(IMAGE_SAVE_DIR)
        print(f"Created directory for saving images: {IMAGE_SAVE_DIR}")

    mode = choose_mode()

    if mode == "q":
        return

    print("\nInitializing Edge Impulse model for Plant Identification...")
    runner = None

    try:
        dir_path = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(dir_path, MODEL_FILENAME)
        runner = ImageImpulseRunner(model_path)
        runner.init()
    except Exception as e:
        print(f"Error initializing Edge Impulse runner: {e}")
        return

    video = cv2.VideoCapture(0)
    video.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not video.isOpened():
        print("Error: Could not open camera.")
        if runner:
            runner.stop()
        return

    mode_names = {
        "1": "10-second automatic mode",
        "2": "SPACEBAR manual mode",
        "3": "continuous mode",
    }

    print("\n==========================================================")
    print(f"Camera active: {mode_names[mode]}")
    print(f"Each stand uses {FRAMES_PER_STAND} Edge Impulse frames.")
    print("Majority result wins.")
    print(f"Backup sequence is used ONLY when all {FRAMES_PER_STAND} frames are empty.")
    print("There are exactly 12 stands.")
    print("Q quits early.")
    if mode == "2":
        print("SPACE = analyze next stand")
    print("==========================================================")

    try:
        connect_serial()
    except (serial.SerialException, OSError) as exc:
        print(f"SERIAL FAILED! {exc}")
        runner.stop()
        video.release()
        cv2.destroyAllWindows()
        return

    start_time = time.time()
    next_auto_trigger = start_time
    stand_number = 0
    manual_armed = mode != "2"
    quit_requested = False

    try:
        while stand_number < len(BACKUP_SEQUENCE):
            ret, live_frame = video.read()
            if ret:
                cv2.imshow("Camera View", live_frame)
            else:
                print("Camera read failed; check camera connection.")
                break
            now = time.time()

            if now - start_time >= DURATION_LIMIT:
                print("\n--- 2 Minute Time Limit Reached. ---")
                break

            # Keep the camera window responsive between stand triggers.
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                print("\n--- User Quit Early ---")
                break

            # ----- Decide whether a new stand should be analyzed -----
            trigger = False

            if mode == "1":
                if now >= next_auto_trigger:
                    trigger = True

            elif mode == "2":
                if key == 32:  # SPACE
                    trigger = True

            elif mode == "3":
                trigger = True

            if not trigger:
                # Show a small live status while waiting.
                wait_img = np.zeros((180, 640, 3), dtype=np.uint8)

                if mode == "1":
                    remaining = max(0.0, next_auto_trigger - now)
                    wait_text = f"Next stand in {remaining:.1f}s"
                elif mode == "2":
                    wait_text = "Press SPACE for next stand"
                else:
                    wait_text = "Ready for next stand"

                cv2.putText(
                    wait_img,
                    f"Stand {stand_number + 1}/12",
                    (30, 65),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (255, 255, 255),
                    3,
                )
                cv2.putText(
                    wait_img,
                    wait_text,
                    (30, 125),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.85,
                    (255, 255, 255),
                    2,
                )
                cv2.imshow("Plant Status & Stand Count", wait_img)

                time.sleep(0.01)
                continue

            # ----- New stand triggered -----
            stand_number += 1

            final_type, final_objects, details, quit_requested = run_single_stand(
                runner,
                video,
                stand_number,
                IMAGE_SAVE_DIR,
            )

            if quit_requested:
                print("\n--- User Quit Early ---")
                break

            show_result(
                final_type,
                final_objects,
                details,
                stand_number,
            )

            # One movement command per completed stand, using the final vote.
            move_for_plant(final_type)

            # Automatic mode timing is measured between stand triggers.
            if mode == "1":
                next_auto_trigger = time.time() + AUTO_TRIGGER_SECONDS

            # Continuous mode immediately proceeds to the next stand.
            # Manual mode waits for another SPACE.

    except KeyboardInterrupt:
        print("\n--- Keyboard Interrupt ---")
    except (RuntimeError, TimeoutError) as exc:
        print(f"\n[STOP] Navigation error: {exc}")

    finally:
        emergency_halt()
        if arduino_nav is not None and arduino_nav.is_open:
            arduino_nav.close()
        if runner:
            runner.stop()

        video.release()
        cv2.destroyAllWindows()

        print("\n=====================================")
        print("         FINAL STAND COUNT")
        print(f"        S: {stand_s} , D: {stand_d} , E: {stand_e}")
        print("=====================================\n")


if __name__ == "__main__":
    run_identification()
