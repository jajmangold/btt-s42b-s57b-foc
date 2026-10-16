#!/usr/bin/env python3
"""Gamepad jog pendant for the drive.

    python tools/pendant.py COM10

Reads any connected game controller and jogs the motor by sending the drive's
own UART commands -- the same `D4 <v>` you would type by hand. Point it at a
serial port connected to the drive's P7 console, directly or through the
ESP32 bridge in tools/esp32-bridge/.

WHY THIS IS SAFE TO DO OVER A LINK
----------------------------------
Jogging is the one case where the link sits in the control path, so the drive
deadman-gates it: motion continues only while fresh jog commands keep
arriving, and stops within 150 ms otherwise. Unplug the controller, close this
script, or let the laptop sleep, and the axis stops on its own.

This script leans on that deliberately. Inside the stick deadzone it sends
NOTHING rather than sending a zero, so every return to centre re-proves the
watchdog instead of exercising it only when something has already gone wrong.

A hardware E-stop still covers what a watchdog cannot.
"""
import os
import sys
import time

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

MAX_JOG_RAD_S = 6.0     # at full stick deflection; a pendant positions, it does not race
DEADZONE      = 0.12
SEND_HZ       = 25      # comfortably inside the drive's 150 ms deadman


def main(port: str, baud: int = 115200) -> int:
    import pygame
    import serial

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No controller found. Pair it over Bluetooth or plug it in, then retry.")
        return 1
    pad = pygame.joystick.Joystick(0)
    pad.init()
    print("controller : %s" % pad.get_name())

    ser = serial.Serial(port, baud, timeout=0.05)
    time.sleep(0.3)
    ser.reset_input_buffer()
    print("drive      : %s" % port)
    print()
    print("  left stick Y   jog")
    print("  A              align")
    print("  B              hold")
    print("  X              off")
    print("  Ctrl-C         quit (the drive stops on its own deadman)")
    print()

    def send(cmd):
        ser.write((cmd + "\n").encode())
        ser.flush()

    period = 1.0 / SEND_HZ
    last_buttons = {}
    jogging = False
    try:
        while True:
            pygame.event.pump()

            for idx, cmd, label in ((0, "D5", "align"), (1, "D1", "hold"), (2, "D0", "off")):
                if idx < pad.get_numbuttons():
                    down = pad.get_button(idx)
                    if down and not last_buttons.get(idx):
                        send(cmd)
                        print("  %s" % label)
                    last_buttons[idx] = down

            v = -pad.get_axis(1) if pad.get_numaxes() > 1 else 0.0
            v = max(-1.0, min(1.0, v))

            if abs(v) < DEADZONE:
                if jogging:
                    jogging = False
                    print("  centred -- letting the deadman stop it")
                # send nothing: the drive's watchdog does the stopping
            else:
                scaled = (abs(v) - DEADZONE) / (1.0 - DEADZONE)
                rate = scaled * MAX_JOG_RAD_S * (1.0 if v > 0 else -1.0)
                send("D4 %.3f" % rate)
                jogging = True

            waiting = ser.read(4096)
            if waiting:
                text = waiting.decode("utf-8", "replace").strip()
                if text:
                    print("  < %s" % text.replace("\n", " | "))

            time.sleep(period)
    except KeyboardInterrupt:
        print("\nstopping")
        send("D0")
        time.sleep(0.2)
    finally:
        ser.close()
        pygame.quit()
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    sys.exit(main(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 115200))
