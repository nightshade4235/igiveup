#!/usr/bin/env python3
"""mensetmanus.py - Robot 1 integrated Raspberry Pi master.

Pi responsibilities:
  IR, VL53L0X, optional MPU-6050, seven servos, 28BYJ-48/ULN2003 drum,
  medkit-only route state machine, and UART commands.

ESP32 compatibility:
  Uses the protocol in esp32s3_l298n_final.ino exactly:
    <S,sequence,checksum>\n
    <V,sequence,F|B|L|R,speed,duration_ms,distance_mm,checksum>\n
  The Pi refreshes movement commands before the ESP32's timed command expires.
  The ESP32 remains the motor/encoder executor; it does not receive route logic.

LIVE HARDWARE WARNING:
  Replace every None calibration value before a live run. The file deliberately
  refuses to move an uncalibrated servo/stepper, because guessing an angle can
  damage the mechanism. Run `python3 mensetmanus.py --dry-run` for a logic test.
"""
from __future__ import annotations

import argparse
import enum
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

# ================================ GPIO =====================================
# BCM GPIO numbering. Verify against the final physical wiring.
IR_S2, IR_S3, IR_S4 = 27, 22, 23
IR_ACTIVE_LOW = True

RAMP_SERVO = 12
SWEEPER_SERVO = 13
TRAPDOOR_1_SERVO = 16
TRAPDOOR_2_SERVO = 18
# Revised wiring PDF, sheet 03:
# SV5 = medical-kit release/red compartment: physical P35 / BCM19.
# SV6 = extra compartment 1/yellow 1: physical P18 / BCM24.
# SV7 = extra compartment 2/yellow 2: physical P22 / BCM25.
RED_KIT_SERVO = 19       # SV5, Pi physical pin 35
YELLOW1_SERVO = 24       # SV6, Pi physical pin 18
YELLOW2_SERVO = 25       # SV7, Pi physical pin 22

STEPPER_IN1, STEPPER_IN2, STEPPER_IN3, STEPPER_IN4 = 4, 5, 6, 20
# Revised wiring PDF, sheet 04: Pi P08/BCM14 TX -> ESP GPIO18 RX;
# Pi P10/BCM15 RX <- ESP GPIO21 TX; Pi P06 GND -> ESP GND.
UART_DEVICE, UART_BAUD = "/dev/serial0", 115200

# ============================== I2C ========================================
MPU_ADDRESS = 0x68
TOF_ADDRESS = 0x29
COLOR_ADDRESS = 0x29
COLOR_BUS_NUMBER = 3
TOF_MIN_VALID_MM, TOF_MAX_VALID_MM = 30.0, 2000.0
TOF_FILTER_WINDOW = 5

# ========================== route calibration =============================
TOF_CLOSE_MM: Optional[float] = 58.0
TOF_CONFIRM_SAMPLES = 1
WHEEL_DIAMETER_MM: Optional[float] = 45.0
DRIVE_MM_PER_SECOND: Optional[float] = 76.85  # provisional loaded test value
ENCODER_COUNTS_PER_MM: Optional[float] = None

FIRST_PATIENT_DISTANCE_MM = 255.0
POST_FIRST_GROUP_REVERSE_MM = 150.0
POST_SECOND_TOF_REVERSE_SECONDS = 1.0
RED_AREA_FORWARD_MM = 220.0
BOARD_CROSSING_DISTANCE_MM = 552.0  # provisional; geometric estimate may differ
MEDKIT_REVERSE_MM = 50.0
MEDKIT_REVERSE_SECONDS = 1.0
MEDKIT_FORWARD_1_SECONDS = 3.0
MEDKIT_FORWARD_2_SECONDS = 3.0
MEDKIT_STRAIGHTEN_FORWARD_SECONDS = 1.0
TURN_90_SECONDS: Optional[float] = None
TURN_180_SECONDS: Optional[float] = None
LEFT_SEARCH_SPEED = 0.22
FORWARD_SPEED = 0.25
SLOW_SPEED = 0.12
REVERSE_SPEED = 0.22

# Four drum slots: Yellow 1, Yellow 2, Red, Green.
SLOT_YELLOW1, SLOT_YELLOW2, SLOT_RED, SLOT_GREEN = 1, 2, 3, 4
INITIAL_DRUM_SLOT = 1
STEPPER_STEPS_PER_SLOT: Optional[int] = None
STEPPER_STEP_DELAY_SEC = 0.002
STEPPER_INVERTED = False

# =========================== servo calibration =============================
# Fill these from real mechanism tests. Values are degrees 0..180.
SERVO_ANGLES = {
    "ramp_up": None, "ramp_down": None,
    "sweeper_home": None, "sweeper_in": None,
    "trapdoor_1_closed": None, "trapdoor_1_open": None,
    "trapdoor_2_closed": None, "trapdoor_2_open": None,
    "yellow1_closed": None, "yellow1_open": None,
    "red_closed": None, "red_open": None,
    "yellow2_closed": None, "yellow2_open": None,
}

# ============================ camera tuning ================================
# HSV ranges are intentionally broad; colour score weights favour saturated
# red/green/yellow samples and reject low-saturation shadow/grey regions.
CAMERA_SIZE = (640, 480)
CAMERA_FPS = 20
CAMERA_MIN_AREA = 180
CAMERA_MIN_SATURATION = 70
CAMERA_MIN_VALUE = 45
CAMERA_SHADOW_SATURATION = 45
CAMERA_CENTER_TOLERANCE_PX = 35
CAMERA_MAX_TURN_CORRECTION = 0.22
COLOR_CONFIDENCE_MIN = 0.34

# TCS34725 starting points; calibrate under competition lighting.
COLOR_MIN_CLEAR = 20
COLOR_WHITE_CLEAR = 1200
COLOR_MARGIN = 0.08
YELLOW_MIN_R, YELLOW_MIN_G, YELLOW_MAX_B = 0.35, 0.30, 0.22

# ============================== protocol ===================================
def checksum(body: str) -> str:
    value = 0
    for b in body.encode("ascii"):
        value ^= b
    return f"{value:02X}"


def frame(body: str) -> bytes:
    return f"<{body},{checksum(body)}>\n".encode("ascii")


# ============================== sensors ====================================
class IRReader:
    def __init__(self, dry: bool):
        self.dry = dry
        self.sensors = []
        if not dry:
            from gpiozero import Button
            self.sensors = [Button(p, pull_up=True) for p in (IR_S2, IR_S3, IR_S4)]

    def read(self) -> tuple[int, int, int]:
        if self.dry:
            return (0, 0, 0)
        out = []
        for s in self.sensors:
            electrical = bool(s.is_pressed)
            out.append(int(electrical if IR_ACTIVE_LOW else not electrical))
        return tuple(out)

    def all_black(self) -> bool:
        return all(self.read())

    def close(self):
        for s in self.sensors:
            s.close()


def classify_color(red: int, green: int, blue: int, clear: int) -> str:
    if clear < COLOR_MIN_CLEAR:
        return "unknown"
    if clear >= COLOR_WHITE_CLEAR:
        return "white"
    total = red + green + blue
    if total <= 0:
        return "unknown"
    r, g, b = red / total, green / total, blue / total
    if r >= YELLOW_MIN_R and g >= YELLOW_MIN_G and b <= YELLOW_MAX_B:
        return "yellow"
    if r >= g + COLOR_MARGIN and r >= b + COLOR_MARGIN:
        return "red"
    if g >= r + COLOR_MARGIN and g >= b + COLOR_MARGIN:
        return "green"
    return "unknown"


class Sensors:
    def __init__(self, dry: bool):
        self.dry = dry
        self.ir = IRReader(dry)
        self.tof = self.color = None
        self.tof_history = deque(maxlen=TOF_FILTER_WINDOW)
        if dry:
            return
        import board, busio, adafruit_vl53l0x
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.tof = adafruit_vl53l0x.VL53L0X(self.i2c, address=TOF_ADDRESS)
        try:
            self.tof.measurement_timing_budget = 200000
            self.tof.signal_rate_limit = 0.1
        except (AttributeError, ValueError):
            pass
        try:
            from adafruit_extended_bus import ExtendedI2C
            import adafruit_tcs34725
            self.color = adafruit_tcs34725.TCS34725(
                ExtendedI2C(COLOR_BUS_NUMBER), address=COLOR_ADDRESS)
        except Exception as exc:
            raise RuntimeError("TCS34725 bus-3 setup failed; verify /dev/i2c-3") from exc

    def tof_mm(self) -> Optional[float]:
        if self.dry or self.tof is None:
            return None
        for _ in range(3):
            try:
                value = float(self.tof.range)
                if TOF_MIN_VALID_MM <= value <= TOF_MAX_VALID_MM:
                    self.tof_history.append(value)
                    a = sorted(self.tof_history)
                    return a[len(a) // 2]
            except (OSError, RuntimeError, TypeError, ValueError):
                pass
            time.sleep(0.005)
        return None

    def color_name(self) -> str:
        if self.dry or self.color is None:
            return "unknown"
        try:
            r, g, b, c = self.color.color_raw
            return classify_color(r, g, b, c)
        except (OSError, RuntimeError, TypeError, ValueError):
            return "unknown"

    def close(self):
        self.ir.close()


@dataclass
class Candidate:
    color: str
    confidence: float
    x: float
    y: float
    area: float


class PatientCamera:
    """Lightweight HSV detector and signed horizontal centering correction."""
    def __init__(self, dry: bool):
        self.dry = dry
        self.picam = None
        self.cv2 = None
        if not dry:
            try:
                import cv2
                from picamera2 import Picamera2
                self.cv2 = cv2
                self.picam = Picamera2()
                cfg = self.picam.create_video_configuration(
                    main={"size": CAMERA_SIZE, "format": "RGB888"})
                self.picam.configure(cfg)
                self.picam.start()
            except Exception as exc:
                raise RuntimeError("Camera startup failed") from exc

    def detect(self) -> Optional[Candidate]:
        if self.dry or self.picam is None:
            return None
        frame_rgb = self.picam.capture_array()
        cv2 = self.cv2
        hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
        h, s, v = cv2.split(hsv)
        valid = (s >= CAMERA_MIN_SATURATION) & (v >= CAMERA_MIN_VALUE)
        # Explicitly reject most grey/black/white shadow pixels.
        valid &= ~((s < CAMERA_SHADOW_SATURATION) | (v < CAMERA_MIN_VALUE))
        masks = {
            "red": ((h < 12) | (h > 170)) & valid,
            "yellow": (h >= 18) & (h <= 42) & valid,
            "green": (h >= 38) & (h <= 92) & valid,
        }
        best = None
        height, width = h.shape[:2]
        for name, mask in masks.items():
            mask8 = (mask.astype("uint8") * 255)
            mask8 = cv2.morphologyEx(mask8, cv2.MORPH_OPEN, None)
            mask8 = cv2.morphologyEx(mask8, cv2.MORPH_CLOSE, None)
            count, labels, stats, cents = cv2.connectedComponentsWithStats(mask8)
            for i in range(1, count):
                area = float(stats[i, cv2.CC_STAT_AREA])
                if area < CAMERA_MIN_AREA:
                    continue
                x, y = cents[i]
                roi_s = float(s[labels == i].mean()) / 255.0
                roi_v = float(v[labels == i].mean()) / 255.0
                # Saturation and compact area create a wavefunction-like score:
                # pixels likely to be real coloured targets get more weight.
                confidence = min(1.0, (area / 2500.0)) * (0.65 * roi_s + 0.35 * roi_v)
                if confidence < COLOR_CONFIDENCE_MIN:
                    continue
                item = Candidate(name, confidence, float(x), float(y), area)
                if best is None or item.confidence > best.confidence:
                    best = item
        return best

    def correction(self, candidate: Optional[Candidate]) -> tuple[float, float]:
        if candidate is None:
            return (0.0, 0.0)
        width = CAMERA_SIZE[0]
        error = (candidate.x - width / 2) / (width / 2)
        if abs(error) < CAMERA_CENTER_TOLERANCE_PX / (width / 2):
            return (0.0, 0.0)
        turn = max(-CAMERA_MAX_TURN_CORRECTION,
                   min(CAMERA_MAX_TURN_CORRECTION, error * CAMERA_MAX_TURN_CORRECTION))
        # Positive image error means target is right: curve right.
        return (FORWARD_SPEED - turn, FORWARD_SPEED + turn)

    def close(self):
        if self.picam is not None:
            self.picam.stop()


# ============================== actuators ==================================
class ServoBank:
    def __init__(self, dry: bool):
        self.dry = dry
        self.servos = {}
        pins = {"ramp": RAMP_SERVO, "sweeper": SWEEPER_SERVO,
                "trap1": TRAPDOOR_1_SERVO, "trap2": TRAPDOOR_2_SERVO,
                "yellow1": YELLOW1_SERVO, "red": RED_KIT_SERVO,
                "yellow2": YELLOW2_SERVO}
        if not dry:
            from gpiozero import AngularServo
            self.servos = {n: AngularServo(p, min_angle=0, max_angle=180,
                initial_angle=None, min_pulse_width=.0005, max_pulse_width=.0025)
                for n, p in pins.items()}

    def move(self, servo: str, angle_key: str):
        angle = SERVO_ANGLES[angle_key]
        if angle is None:
            raise RuntimeError(f"Set SERVO_ANGLES['{angle_key}'] before live operation")
        if not self.dry:
            self.servos[servo].angle = angle
        print(f"SERVO {servo} -> {angle_key} ({angle})")

    def pickup(self):
        self.move("sweeper", "sweeper_in")
        time.sleep(.15)
        self.move("ramp", "ramp_up")
        time.sleep(.20)
        self.move("sweeper", "sweeper_home")
        self.move("ramp", "ramp_down")

    def trapdoors(self, opened: bool):
        self.move("trap1", "trapdoor_1_open" if opened else "trapdoor_1_closed")
        self.move("trap2", "trapdoor_2_open" if opened else "trapdoor_2_closed")

    def release(self, which: str):
        self.move(which, f"{which}_open")
        time.sleep(.25)
        self.move(which, f"{which}_closed")

    def close(self):
        for s in self.servos.values():
            s.detach()


class DrumStepper:
    SEQ = ((1,0,0,0),(1,1,0,0),(0,1,0,0),(0,1,1,0),
           (0,0,1,0),(0,0,1,1),(0,0,0,1),(1,0,0,1))
    def __init__(self, dry: bool):
        self.dry, self.current_slot, self.out = dry, INITIAL_DRUM_SLOT, []
        if not dry:
            from gpiozero import DigitalOutputDevice
            self.out = [DigitalOutputDevice(p, initial_value=False)
                        for p in (STEPPER_IN1, STEPPER_IN2, STEPPER_IN3, STEPPER_IN4)]

    def write(self, pat):
        for o, v in zip(self.out, pat): o.value = bool(v)

    def move_to(self, target: int):
        if STEPPER_STEPS_PER_SLOT is None:
            raise RuntimeError("Set STEPPER_STEPS_PER_SLOT before live drum movement")
        delta = (target - self.current_slot) % 4
        step_sign = -1 if STEPPER_INVERTED else 1
        seq = self.SEQ[::step_sign]
        for i in range(delta * STEPPER_STEPS_PER_SLOT):
            self.write(seq[i % len(seq)])
            time.sleep(STEPPER_STEP_DELAY_SEC)
        self.write((0,0,0,0))
        self.current_slot = target
        print(f"DRUM -> slot {target} (delta {delta})")

    def close(self):
        self.write((0,0,0,0))
        for o in self.out: o.close()


# =============================== UART ======================================
class ESPLink:
    def __init__(self, dry: bool):
        self.dry, self.seq, self.port = dry, 0, None
        if not dry:
            import serial
            self.port = serial.Serial(UART_DEVICE, UART_BAUD, timeout=.01)

    def send(self, body: str):
        packet = frame(body)
        if self.port: self.port.write(packet)
        print("UART TX", packet.decode().strip())

    def drive(self, left: float, right: float, duration_ms: int = 180):
        self.seq += 1
        left, right = max(-1, min(1, left)), max(-1, min(1, right))
        if abs(left) < .02 and abs(right) < .02:
            self.stop(); return
        if left >= 0 and right >= 0: direction = "F"
        elif left <= 0 and right <= 0: direction = "B"
        elif left < 0 and right > 0: direction = "L"
        else: direction = "R"
        speed = max(abs(left), abs(right))
        self.send(f"V,{self.seq},{direction},{speed:.3f},{int(duration_ms)},0")

    def stop(self):
        self.seq += 1
        self.send(f"S,{self.seq}")

    def read(self):
        if not self.port: return
        while self.port.in_waiting:
            line = self.port.readline().decode("ascii", "replace").strip()
            if line.startswith("<E,"): self.last_encoder = line

    def close(self):
        if self.port: self.port.close()


# ============================ state machine ================================
class State(enum.Enum):
    START = enum.auto(); CROSS_START = enum.auto(); FIRST_APPROACH = enum.auto()
    FIND_PATIENT = enum.auto(); ALIGN_PATIENT = enum.auto(); COLLECT = enum.auto()
    WAIT_FIRST_CLOSE = enum.auto(); FIRST_CLOSE_REVERSE = enum.auto(); TURN_AROUND = enum.auto()
    SECOND_GROUP = enum.auto(); LEAVE_PATIENTS = enum.auto(); POST_IR_REVERSE = enum.auto()
    RED_APPROACH = enum.auto(); RED_DISPENSE = enum.auto(); TO_POSITION1 = enum.auto()
    YELLOW_DISPENSE = enum.auto(); RETURN_POSITION1 = enum.auto(); CROSS_BOARD = enum.auto()
    FINAL_LEFT_TOF = enum.auto(); FINAL_RIGHT_IR = enum.auto(); FINAL_APPROACH = enum.auto()
    GREEN_DISPENSE = enum.auto(); FINISHED = enum.auto(); FAULT = enum.auto()
    MEDKIT_TRANSIT = enum.auto(); MEDKIT_REVERSE_50 = enum.auto()
    MEDKIT_TURN_LEFT = enum.auto(); MEDKIT_REVERSE_1 = enum.auto()
    MEDKIT_SV5 = enum.auto(); MEDKIT_FORWARD_1 = enum.auto()
    MEDKIT_SV7 = enum.auto(); MEDKIT_FORWARD_2 = enum.auto()
    MEDKIT_STRAIGHTEN_TURN = enum.auto(); MEDKIT_STRAIGHTEN_FORWARD = enum.auto()
    MEDKIT_SV6 = enum.auto()


class PatientRobot:
    def __init__(self, dry: bool = False):
        self.dry = dry
        self.sensors = Sensors(dry)
        # The current competition route is medkit-only; camera startup is
        # intentionally disabled and can be restored for patient mode later.
        self.camera = None
        self.servos, self.drum, self.esp = ServoBank(dry), DrumStepper(dry), ESPLink(dry)
        self.state = State.START
        self.state_since = time.monotonic()
        self.phase_end: Optional[float] = None
        self.total_count = 0
        self.group_count = 0
        self.yellow_count = 0
        self.black_streak = 0
        self.close_streak = 0
        self.candidate: Optional[Candidate] = None
        self.collected_this_state = False
        self.running = True

    def set_state(self, state: State):
        self.state, self.state_since = state, time.monotonic()
        self.phase_end = None
        self.black_streak = self.close_streak = 0
        self.collected_this_state = False
        print(f"STATE -> {state.name}")

    def all_black(self, samples: int = 3) -> bool:
        self.black_streak = self.black_streak + 1 if self.sensors.ir.all_black() else 0
        return self.black_streak >= samples

    def tof_close(self) -> bool:
        v = self.sensors.tof_mm()
        self.close_streak = self.close_streak + 1 if v is not None and TOF_CLOSE_MM is not None and v <= TOF_CLOSE_MM else 0
        return self.close_streak >= TOF_CONFIRM_SAMPLES

    def drive_distance(self, left: float, right: float, mm: float):
        if DRIVE_MM_PER_SECOND is None:
            raise RuntimeError("Set DRIVE_MM_PER_SECOND before distance movement")
        if self.phase_end is None:
            self.phase_end = time.monotonic() + abs(mm) / DRIVE_MM_PER_SECOND
        self.esp.drive(left, right)
        return time.monotonic() >= self.phase_end

    def timed(self, left: float, right: float, seconds: float):
        if self.phase_end is None: self.phase_end = time.monotonic() + seconds
        self.esp.drive(left, right)
        return time.monotonic() >= self.phase_end

    def turn(self, right: bool, degrees: float):
        duration = TURN_180_SECONDS if degrees >= 180 else TURN_90_SECONDS
        if duration is None:
            raise RuntimeError("Set TURN_90_SECONDS/TURN_180_SECONDS before live turns")
        return self.timed(FORWARD_SPEED if right else -FORWARD_SPEED,
                          -FORWARD_SPEED if right else FORWARD_SPEED, duration)

    def pickup_patient(self):
        self.esp.stop()
        detected = self.sensors.color_name()
        if detected not in ("red", "green", "yellow") and self.candidate:
            detected = self.candidate.color
        if detected not in ("red", "green", "yellow"):
            print("COLOUR UNKNOWN; retrying pickup without counting")
            return False
        self.servos.pickup()
        self.total_count += 1
        self.group_count += 1
        if detected == "yellow": self.yellow_count += 1
        slot = {"red": SLOT_RED, "green": SLOT_GREEN,
                "yellow": SLOT_YELLOW1 if self.yellow_count <= 2 else SLOT_YELLOW2}[detected]
        self.drum.move_to(slot)
        print(f"COLLECTED colour={detected} total={self.total_count} group={self.group_count} yellow={self.yellow_count}")
        return True

    def dispense(self, slot: int, kit_servo: Optional[str] = None):
        self.esp.stop(); self.drum.move_to(slot); self.servos.trapdoors(True)
        if kit_servo: self.servos.release(kit_servo)
        self.servos.trapdoors(False)

    def tick(self):
        self.esp.read()
        s = self.state
        if s == State.START:
            self.drum.current_slot = INITIAL_DRUM_SLOT
            self.set_state(State.CROSS_START)
        elif s == State.CROSS_START:
            self.esp.drive(FORWARD_SPEED, FORWARD_SPEED)
            if self.all_black(): self.set_state(State.MEDKIT_TRANSIT)
        elif s == State.MEDKIT_TRANSIT:
            # Starting-zone line has already been crossed. Continue to the
            # next landmark: either a valid ToF close reading or a new
            # all-black landmark event.
            self.esp.drive(FORWARD_SPEED, FORWARD_SPEED)
            if self.tof_close() or self.all_black():
                self.set_state(State.MEDKIT_REVERSE_50)
        elif s == State.MEDKIT_REVERSE_50:
            if self.drive_distance(-REVERSE_SPEED, -REVERSE_SPEED, MEDKIT_REVERSE_MM):
                self.set_state(State.MEDKIT_TURN_LEFT)
        elif s == State.MEDKIT_TURN_LEFT:
            # "Go to the left" is implemented as a calibrated left 90-degree
            # turn. The robot is then straightened by the following motions.
            if self.turn(False, 90): self.set_state(State.MEDKIT_REVERSE_1)
        elif s == State.MEDKIT_REVERSE_1:
            if self.timed(-REVERSE_SPEED, -REVERSE_SPEED, MEDKIT_REVERSE_SECONDS):
                self.set_state(State.MEDKIT_SV5)
        elif s == State.MEDKIT_SV5:
            self.esp.stop()
            self.servos.release("yellow1")  # SV5: right-side Yellow 1
            self.set_state(State.MEDKIT_FORWARD_1)
        elif s == State.MEDKIT_FORWARD_1:
            if self.timed(FORWARD_SPEED, FORWARD_SPEED, MEDKIT_FORWARD_1_SECONDS):
                self.set_state(State.MEDKIT_SV7)
        elif s == State.MEDKIT_SV7:
            self.esp.stop()
            self.servos.release("red")  # SV7: left-side red medkits
            self.set_state(State.MEDKIT_FORWARD_2)
        elif s == State.MEDKIT_FORWARD_2:
            if self.timed(FORWARD_SPEED, FORWARD_SPEED, MEDKIT_FORWARD_2_SECONDS):
                self.set_state(State.MEDKIT_STRAIGHTEN_TURN)
        elif s == State.MEDKIT_STRAIGHTEN_TURN:
            if self.turn(False, 90): self.set_state(State.MEDKIT_STRAIGHTEN_FORWARD)
        elif s == State.MEDKIT_STRAIGHTEN_FORWARD:
            if self.timed(FORWARD_SPEED, FORWARD_SPEED, MEDKIT_STRAIGHTEN_FORWARD_SECONDS):
                self.set_state(State.MEDKIT_SV6)
        elif s == State.MEDKIT_SV6:
            self.esp.stop()
            self.servos.release("yellow2")  # SV6: middle Yellow 2
            self.set_state(State.FINISHED)
        elif s == State.FIRST_APPROACH:
            if self.drive_distance(FORWARD_SPEED, FORWARD_SPEED, FIRST_PATIENT_DISTANCE_MM):
                self.esp.stop(); self.set_state(State.FIND_PATIENT)
        elif s == State.FIND_PATIENT:
            self.candidate = self.camera.detect()
            if self.candidate: self.set_state(State.ALIGN_PATIENT)
            else: self.esp.drive(FORWARD_SPEED, FORWARD_SPEED)
        elif s == State.ALIGN_PATIENT:
            self.candidate = self.camera.detect() or self.candidate
            if not self.candidate: self.set_state(State.FIND_PATIENT); return
            left, right = self.camera.correction(self.candidate)
            if left == right == 0: self.set_state(State.COLLECT)
            else: self.esp.drive(left, right)
        elif s == State.COLLECT:
            if not self.collected_this_state:
                self.collected_this_state = True
                if self.pickup_patient():
                    if self.group_count >= 6:
                        self.set_state(State.WAIT_FIRST_CLOSE if self.total_count <= 6 else State.LEAVE_PATIENTS)
                    else:
                        self.set_state(State.FIND_PATIENT)
                else: self.collected_this_state = False
        elif s == State.WAIT_FIRST_CLOSE:
            # First close event ends group one; group_count is reset, yellow_count is not.
            if self.tof_close(): self.set_state(State.FIRST_CLOSE_REVERSE)
            else: self.esp.drive(FORWARD_SPEED, FORWARD_SPEED)
        elif s == State.FIRST_CLOSE_REVERSE:
            if self.drive_distance(-REVERSE_SPEED, -REVERSE_SPEED, POST_FIRST_GROUP_REVERSE_MM):
                self.set_state(State.TURN_AROUND)
        elif s == State.TURN_AROUND:
            if self.turn(True, 180):
                self.group_count = 0; self.set_state(State.FIND_PATIENT)
        elif s == State.LEAVE_PATIENTS:
            if self.tof_close(): self.set_state(State.POST_IR_REVERSE)
            else: self.esp.drive(LEFT_SEARCH_SPEED, LEFT_SEARCH_SPEED)
        elif s == State.POST_IR_REVERSE:
            if self.timed(-REVERSE_SPEED, -REVERSE_SPEED, POST_SECOND_TOF_REVERSE_SECONDS): self.set_state(State.RED_APPROACH)
        elif s == State.RED_APPROACH:
            if self.drive_distance(FORWARD_SPEED, FORWARD_SPEED, RED_AREA_FORWARD_MM): self.set_state(State.RED_DISPENSE)
        elif s == State.RED_DISPENSE:
            self.dispense(SLOT_RED, "red"); self.set_state(State.TO_POSITION1)
        elif s == State.TO_POSITION1:
            self.esp.drive(FORWARD_SPEED, FORWARD_SPEED)
            if self.all_black(): self.set_state(State.YELLOW_DISPENSE)
        elif s == State.YELLOW_DISPENSE:
            if self.turn(False, 90):
                yellow_slot = SLOT_YELLOW2 if self.yellow_count > 2 else SLOT_YELLOW1
                yellow_servo = "yellow2" if yellow_slot == SLOT_YELLOW2 else "yellow1"
                self.dispense(yellow_slot, yellow_servo)
                self.set_state(State.RETURN_POSITION1)
        elif s == State.RETURN_POSITION1:
            self.esp.drive(-REVERSE_SPEED, -REVERSE_SPEED)
            if self.all_black(): self.set_state(State.CROSS_BOARD)
        elif s == State.CROSS_BOARD:
            if self.drive_distance(FORWARD_SPEED, FORWARD_SPEED, BOARD_CROSSING_DISTANCE_MM): self.set_state(State.FINAL_LEFT_TOF)
        elif s == State.FINAL_LEFT_TOF:
            if self.tof_close(): self.set_state(State.FINAL_RIGHT_IR)
            else: self.esp.drive(-FORWARD_SPEED, FORWARD_SPEED)
        elif s == State.FINAL_RIGHT_IR:
            self.esp.drive(FORWARD_SPEED, -FORWARD_SPEED)
            if self.all_black(): self.set_state(State.FINAL_APPROACH)
        elif s == State.FINAL_APPROACH:
            self.esp.drive(SLOW_SPEED, SLOW_SPEED)
            if self.tof_close(): self.set_state(State.GREEN_DISPENSE)
        elif s == State.GREEN_DISPENSE:
            self.dispense(SLOT_GREEN); self.set_state(State.FINISHED)
        elif s == State.FINISHED:
            self.esp.stop(); self.running = False

    def run(self):
        print("mensetmanus.py starting", "DRY RUN" if self.dry else "LIVE")
        try:
            while self.running:
                self.tick(); time.sleep(.05)
        except KeyboardInterrupt:
            print("STOP: keyboard interrupt")
        except Exception as exc:
            print(f"FAULT: {exc}")
            self.state = State.FAULT
        finally:
            self.esp.stop()
            self.drum.close(); self.servos.close()
            if self.camera is not None: self.camera.close()
            self.sensors.close(); self.esp.close()


def main() -> int:
    ap = argparse.ArgumentParser(description="Robot 1 integrated Pi master")
    ap.add_argument("--dry-run", action="store_true", help="run state logic without hardware")
    args = ap.parse_args()
    PatientRobot(dry=args.dry_run).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
