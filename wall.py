#!/usr/bin/env python3
"""Robot 2 wall-only Raspberry Pi master.

Current competition route:
  1. Drive forward until the first all-black IR event: starting zone exit.
  2. Continue forward until the next all-black IR event: disk area reached.
  3. Drive over the disk area without camera/disk detection or disk servos.
  4. Lower the wall mechanism.
  5. Approach the wall slowly until a valid ToF close reading.
  6. Stop and release the walls.

Wiring is the previously issued Robot 2 wiring:
  IR S2/S3/S4 = BCM27/22/23, black = 1.
  VL53L0X = hardware I2C bus 1, address 0x29.
  Servo HAT = hardware I2C, normally 0x40.
  Wall grabbers = HAT channels 6 and 7.
  Wall lift = HAT channel 8.
  Pi UART TX/RX = physical 8/10 (BCM14/15) to ESP GPIO18/21.

The existing ESP32-S3 firmware is unchanged. It receives the same framed
<S,...> and <V,...> commands as esp32s3_l298n_final.ino.
"""
from __future__ import annotations

import argparse
import time
from enum import Enum, auto
from typing import Optional

# ================================ macros ===================================
IR_S2, IR_S3, IR_S4 = 27, 22, 23
IR_BLACK_LEVEL = 1
IR_CONFIRM_SAMPLES = 3
UART_DEVICE, UART_BAUD = "/dev/serial0", 115200

TOF_ADDRESS = 0x29
TOF_MIN_VALID_MM = 30.0
TOF_MAX_VALID_MM = 2000.0
TOF_CLOSE_MM: Optional[float] = 120.0  # calibrate on the real wall
TOF_CONFIRM_SAMPLES = 3
TOF_FILTER_WINDOW = 5

SERVO_HAT_ADDRESS = 0x40  # verify with i2cdetect
WALL_GRAB_LEFT = 6
WALL_GRAB_RIGHT = 7
WALL_LIFT = 8

# Fill these after testing the actual wall mechanism.
SERVO_ANGLES = {
    "wall_grab_open": None,
    "wall_grab_closed": None,
    "wall_lift_up": None,
    "wall_lift_down": None,
}

FORWARD_SPEED = 0.22
WALL_APPROACH_SPEED = 0.10
MOTOR_COMMAND_MS = 180
WALL_SETTLE_SECONDS = 0.25


# =============================== UART ======================================
def checksum(body: str) -> str:
    value = 0
    for b in body.encode("ascii"):
        value ^= b
    return f"{value:02X}"


def frame(body: str) -> bytes:
    return f"<{body},{checksum(body)}>\n".encode("ascii")


class UART:
    def __init__(self, dry: bool):
        self.dry, self.seq, self.ser = dry, 0, None
        if not dry:
            import serial
            self.ser = serial.Serial(UART_DEVICE, UART_BAUD, timeout=0.01, write_timeout=.2)

    def send(self, direction: str, speed: float, duration_ms: int = MOTOR_COMMAND_MS):
        self.seq += 1
        speed = max(0.0, min(1.0, float(speed)))
        packet = frame(f"V,{self.seq},{direction},{speed:.3f},{int(duration_ms)},0")
        if self.ser: self.ser.write(packet)
        print("UART TX", packet.decode().strip())

    def stop(self):
        self.seq += 1
        packet = frame(f"S,{self.seq}")
        if self.ser: self.ser.write(packet)
        print("UART TX", packet.decode().strip())

    def close(self):
        try: self.stop()
        finally:
            if self.ser: self.ser.close()


# ============================== hardware ===================================
class Hardware:
    def __init__(self, dry: bool):
        self.dry = dry
        self.ir = []
        self.tof = None
        self.kit = None
        self.tof_history: list[float] = []
        if dry: return

        from gpiozero import DigitalInputDevice
        self.ir = [DigitalInputDevice(p, pull_up=False) for p in (IR_S2, IR_S3, IR_S4)]

        import board, busio, adafruit_vl53l0x
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.tof = adafruit_vl53l0x.VL53L0X(self.i2c, address=TOF_ADDRESS)
        try:
            self.tof.measurement_timing_budget = 200000
            self.tof.signal_rate_limit = 0.1
        except (AttributeError, ValueError):
            pass

        from adafruit_servokit import ServoKit
        self.kit = ServoKit(channels=16, address=SERVO_HAT_ADDRESS)

    def all_black(self) -> bool:
        values = [int(x.value) for x in self.ir] if self.ir else [0, 0, 0]
        result = all(v == IR_BLACK_LEVEL for v in values)
        print(f"IR S2/S3/S4={values} all_black={int(result)}")
        return result

    def tof_mm(self) -> Optional[float]:
        if self.tof is None:
            return None
        try:
            value = float(self.tof.range)
        except (OSError, RuntimeError, TypeError, ValueError):
            return None
        if not (TOF_MIN_VALID_MM <= value <= TOF_MAX_VALID_MM):
            print(f"ToF invalid={value}")
            return None
        self.tof_history.append(value)
        self.tof_history = self.tof_history[-TOF_FILTER_WINDOW:]
        ordered = sorted(self.tof_history)
        filtered = ordered[len(ordered) // 2]
        print(f"ToF raw={value:.0f} filtered={filtered:.0f} mm")
        return filtered

    def tof_close(self) -> bool:
        value = self.tof_mm()
        return TOF_CLOSE_MM is not None and value is not None and value <= TOF_CLOSE_MM

    def servo(self, channel: int, key: str):
        angle = SERVO_ANGLES[key]
        if angle is None:
            raise RuntimeError(f"Set SERVO_ANGLES['{key}'] before live operation")
        if self.kit: self.kit.servo[channel].angle = angle
        print(f"SERVO HAT ch{channel} -> {key} ({angle} deg)")

    def walls_up(self):
        self.servo(WALL_LIFT, "wall_lift_up")
        self.servo(WALL_GRAB_LEFT, "wall_grab_open")
        self.servo(WALL_GRAB_RIGHT, "wall_grab_open")

    def walls_lower(self):
        # Lower the carrier while keeping the wall grippers closed until the
        # robot has reached the final wall position.
        self.servo(WALL_GRAB_LEFT, "wall_grab_closed")
        self.servo(WALL_GRAB_RIGHT, "wall_grab_closed")
        self.servo(WALL_LIFT, "wall_lift_down")

    def walls_release(self):
        self.servo(WALL_GRAB_LEFT, "wall_grab_open")
        self.servo(WALL_GRAB_RIGHT, "wall_grab_open")

    def close(self):
        for device in self.ir: device.close()


# ============================== route ======================================
class State(Enum):
    START = auto()
    EXIT_START = auto()
    WAIT_SECOND_BLACK = auto()
    CROSS_DISKS = auto()
    LOWER_WALLS = auto()
    APPROACH_WALL = auto()
    RELEASE_WALLS = auto()
    FINISHED = auto()


class Robot2WallOnly:
    def __init__(self, dry: bool = False):
        self.dry = dry
        self.hw = Hardware(dry)
        self.uart = UART(dry)
        self.state = State.START
        self.black_streak = 0
        self.close_streak = 0
        self.last_black = False
        self.state_since = time.monotonic()
        self.running = True

    def set_state(self, state: State):
        self.state = state
        self.state_since = time.monotonic()
        self.black_streak = 0
        self.close_streak = 0
        print("STATE ->", state.name)

    def black_event(self) -> bool:
        now_black = self.hw.all_black()
        self.black_streak = self.black_streak + 1 if now_black else 0
        return self.black_streak >= IR_CONFIRM_SAMPLES

    def clear_black(self, timeout_s: float = 3.0):
        start = time.monotonic()
        while self.hw.all_black() and time.monotonic() - start < timeout_s:
            self.uart.send("F", FORWARD_SPEED)
            time.sleep(.05)

    def close_event(self) -> bool:
        self.close_streak = self.close_streak + 1 if self.hw.tof_close() else 0
        return self.close_streak >= TOF_CONFIRM_SAMPLES

    def tick(self):
        if self.state == State.START:
            self.hw.walls_up()
            self.set_state(State.EXIT_START)

        elif self.state == State.EXIT_START:
            self.uart.send("F", FORWARD_SPEED)
            if self.black_event():
                self.uart.stop()
                self.clear_black()
                self.set_state(State.WAIT_SECOND_BLACK)

        elif self.state == State.WAIT_SECOND_BLACK:
            # This is the approach to / crossing of the disk region. Disks are
            # deliberately ignored; only the next IR landmark matters.
            self.uart.send("F", FORWARD_SPEED)
            if self.black_event():
                self.uart.stop()
                self.set_state(State.CROSS_DISKS)

        elif self.state == State.CROSS_DISKS:
            # No camera, disk detector, disk servos, or disk stopping logic.
            # Continue over the disk area until the wall mechanism is lowered.
            self.uart.send("F", FORWARD_SPEED)
            if time.monotonic() - self.state_since >= 0.5:
                self.set_state(State.LOWER_WALLS)

        elif self.state == State.LOWER_WALLS:
            self.uart.stop()
            self.hw.walls_lower()
            time.sleep(WALL_SETTLE_SECONDS)
            self.set_state(State.APPROACH_WALL)

        elif self.state == State.APPROACH_WALL:
            self.uart.send("F", WALL_APPROACH_SPEED)
            if self.close_event():
                self.uart.stop()
                self.set_state(State.RELEASE_WALLS)

        elif self.state == State.RELEASE_WALLS:
            self.uart.stop()
            self.hw.walls_release()
            time.sleep(WALL_SETTLE_SECONDS)
            self.set_state(State.FINISHED)

        elif self.state == State.FINISHED:
            self.uart.stop()
            self.running = False

    def run(self):
        print("Robot 2 wall-only master starting", "DRY RUN" if self.dry else "LIVE")
        try:
            while self.running:
                self.tick()
                time.sleep(.05)
        except KeyboardInterrupt:
            print("STOP: keyboard interrupt")
        except Exception as exc:
            print("FAULT:", exc)
            self.state = State.FINISHED
        finally:
            self.uart.close()
            self.hw.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Robot 2 wall-only Pi master")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    Robot2WallOnly(args.dry_run).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
