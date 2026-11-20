#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
  Unitree Go1 Air — PS4 DualShock 4 Controller Teleoperation
================================================================================
  File    : unitree_go1_ps4_controller.py
  Target  : Raspberry Pi (pi@192.168.12.1) on Unitree Go1 Air
  Protocol: Direct High-Level UDP SDK @ 192.168.123.161:8082 (50 Hz, zero MQTT)

--------------------------------------------------------------------------------
  OFFICIAL UNITREE GO1 HighCmd MODES & FSM
--------------------------------------------------------------------------------
  0: idle (default stand / power-on)
  1: force stand (controlled by bodyHeight + YPR, transition from 2 or 6)
  2: target velocity walking (controlled by vx, vy, vyaw, transition from 1)
  5: position stand down (transitional kinematic descent, must NOT be held)
  6: position stand up (transitional kinematic ascent, transition from 1 or 5)
  7: damping mode (all motors limp, soft e-stop, transition from any mode)
  8: recovery stand (from mode 7 only)

  OFFICIAL SEQUENCES:
  - Sit Down : 1 (0.5 s) -> 6 (1.0 s) -> 5 (2.0 s) -> 7 (1.0 s) -> ends in DAMP
  - Stand Up : 7 (0.5 s) -> 5 (1.0 s) -> 6 (2.0 s) -> 1 (1.0 s) -> ends in STAND

--------------------------------------------------------------------------------
  MODES & BUTTON MAPPING
--------------------------------------------------------------------------------
  BUTTON COMBO         MODE          DESCRIPTION
  -------------------  ------------ -----------------------------------------
  L2 + TRIANGLE (▲)   WALK          Trot gait, 8 cm foot clearance (gaitType 1)
  L2 + CROSS    (✖)   STAIRS        Climb stair crawl (gaitType 3, 8 cm clearance, closed-loop heading hold)
  L2 + SQUARE   (■)   STAND         Joints locked, Right-Stick pitch tilt only
                                    Also used to stand up from DAMP
  L2 + CIRCLE   (⭕)   SIT DOWN      Smooth 4-stage descent (1->6->5->7) → DAMP (from STAND)
  D-Pad UP            HEIGHT +      Continuous body height raise (STAND mode only, max +0.08 m)
  D-Pad DOWN          HEIGHT -      Continuous body height lower (STAND mode only, min -0.10 m)

--------------------------------------------------------------------------------
  STICK MAPPING
--------------------------------------------------------------------------------
  Left  Stick Y  — Forward (+) / Backward (-)         [WALK, STAIRS]
  Left  Stick X  — Strafe Left (+) / Right (-)        [WALK only]
  Right Stick X  — Yaw Turn [WALK: ±2.5 rad/s, STAIRS: ±0.5 rad/s with heading hold]
  Right Stick Y  — Pitch Tilt Up / Down               [STAND only]
                   ±0.35 rad (±20°), spring auto-centering to 0.0 rad

--------------------------------------------------------------------------------
  VERIFIED VELOCITY LIMITS & CONTROL DYNAMICS
--------------------------------------------------------------------------------
  Walk forward      : + 1.50 m/s  (trot gait, smooth slew rate ramp)
  Walk backward     : - 1.00 m/s  (trot gait reverse)
  Walk strafe       : ± 1.00 m/s  (lateral strafe)
  Walk yaw          : ± 2.50 rad/s (agile turning rate)
  Stair forward     : + 0.20 m/s  (climb stair ascent crawl)
  Stair backward    : - 0.15 m/s  (climb stair descent crawl)
  Stair strafe      :   0.00 m/s  (locked for gaitType 3 crawl stability)
  Stair yaw         : ± 0.50 rad/s (Right Stick manual steering override)
  Stair heading hold: Proportional closed-loop yaw tracking (Kp=1.20, clamp=±0.35 rad/s)
  Height ramp speed :   0.12 m/s  (smooth continuous body height ramping in STAND)

================================================================================
"""

import os
import sys
import time
import math
import glob
import struct
import signal
import socket
import logging

# ------------------------------------------------------------------------------
# LOGGING SETUP — clean, readable console output
# ------------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("Go1Teleop")

# ------------------------------------------------------------------------------
# IMPORT UNITREE HIGH-LEVEL UDP SDK & BMS EXTENSION
# ------------------------------------------------------------------------------
BUILD_PATH = "/home/pi/Unitree/autostart/programming/build"
PS4_PATH   = "/home/pi/Unitree/ps4_controller"
LOCAL_PATH = os.path.dirname(os.path.abspath(__file__))

for p in (BUILD_PATH, PS4_PATH, LOCAL_PATH):
    if p not in sys.path:
        sys.path.append(p)

# Import pybind11 BMS type definition so st.bms.SOC is properly registered
try:
    import unitree_bms
    logger.info("Unitree BMS telemetry module registered.")
except ImportError:
    logger.warning("unitree_bms module not found; BMS types may not be bound.")

try:
    import robot_interface_high_level as sdk
    logger.info("Unitree High-Level UDP SDK loaded successfully.")
except ImportError as err:
    logger.error("Failed to import robot_interface_high_level: %s", err)
    logger.error("Verify that %s exists on the Pi.", BUILD_PATH)
    sys.exit(1)

# ------------------------------------------------------------------------------
# CONSTANTS: TIMING & NETWORK
# ------------------------------------------------------------------------------
LOOP_RATE     = 1.0 / 50.0          # 50 Hz = 20 ms cycle
ROBOT_IP      = "192.168.123.161"
ROBOT_PORT    = 8082
DEFAULT_JS    = "/dev/input/js0"

# ------------------------------------------------------------------------------
# CONSTANTS: LINUX EVDEV / JOYSTICK PS4 MAPPING
# ------------------------------------------------------------------------------
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS   = 0x02
JS_EVENT_INIT   = 0x80

AXIS_LS_X       = 0   # Left  Stick horizontal: -32767 = left,  +32767 = right
AXIS_LS_Y       = 1   # Left  Stick vertical  : -32767 = up,    +32767 = down
AXIS_L2         = 2   # L2 trigger analog     : -32767 = released, +32767 = pulled
AXIS_RS_X       = 3   # Right Stick horizontal: -32767 = left,  +32767 = right
AXIS_RS_Y       = 4   # Right Stick vertical  : -32767 = up,    +32767 = down
AXIS_R2         = 5   # R2 trigger analog     : -32767 = released, +32767 = pulled
AXIS_DPAD_X     = 6   # D-Pad horizontal      : -32767 = left,  +32767 = right
AXIS_DPAD_Y     = 7   # D-Pad vertical        : -32767 = up,    +32767 = down

BTN_CROSS       = 0   # ✖  (bottom)
BTN_CIRCLE      = 1   # ⭕  (right)
BTN_TRIANGLE    = 2   # ▲  (top)
BTN_SQUARE      = 3   # ■  (left)
BTN_L1          = 4   # L1 shoulder
BTN_R1          = 5   # R1 shoulder
BTN_L2          = 6   # L2 button click (thresholded trigger)
BTN_R2          = 7   # R2 button click (thresholded trigger)
BTN_SHARE       = 8   # Share / Create
BTN_OPTIONS     = 9   # Options
BTN_PS          = 10  # PS logo button
BTN_L3          = 11  # Left  Stick click
BTN_R3          = 12  # Right Stick click

JS_EVENT_STRUCT = struct.Struct("IhBB")   # 8 bytes: timestamp(4), value(2), type(1), number(1)


# ==============================================================================
# MAIN CONTROLLER CLASS
# ==============================================================================
class Go1PS4Controller:
    """
    Direct 50 Hz UDP teleoperation daemon for Unitree Go1 Air.
    Communicates straight to HighCmd port 8082 with Zero MQTT dependency.
    """

    def __init__(self, js_path=DEFAULT_JS):
        self.js_path   = js_path
        self.js_fd     = None
        self.robot     = None

        # State machine: defaults to DAMP on startup (robot rests on floor)
        self.mode_state = "DAMP"

        # Walking parameters (gaitType 1 = trot, 8 cm foot clearance)
        self.WALK_VX_FWD      = 1.50     # Forward max   : +1.50 m/s
        self.WALK_VX_BWD      = 1.00     # Backward max  : -1.00 m/s
        self.WALK_VY          = 1.00     # Lateral strafe: ±1.00 m/s
        self.WALK_VYAW        = 2.50     # Yaw / turn    : ±2.50 rad/s
        self.WALK_FOOT_RAISE  = 0.08     # 8 cm foot raise
        self.STICK_DEADZONE   = 0.08     # 8% radial deadzone

        # Stair parameters (gaitType 3 = climb stair)
        self.STAIR_VX_FWD     = 0.20     # Stair ascent speed: +0.20 m/s (20 cm/s)
        self.STAIR_VX_BWD     = 0.15     # Stair descent / reverse speed: -0.15 m/s (15 cm/s)
        self.STAIR_VYAW       = 0.50     # Stair yaw steering: ±0.50 rad/s
        self.STAIR_FOOT_RAISE = 0.08     # 8 cm foot raise (clears steps cleanly, avoids stumbling and excessive roll)
        self.STAIR_DEADZONE   = 0.20     # 20% deadzone: larger than STICK_DEADZONE, reduces overshoot jitter
        self.STAIR_ACCEL_UP   = 0.60     # m/s^2 responsive start (0.20 m/s in ~0.33 s)
        self.STAIR_ACCEL_DOWN = 0.60     # m/s^2 smooth stop (0.20 m/s stops in ~0.33 s)
        self.STAIR_YAW_ACCEL  = 1.50     # rad/s^2 stair yaw rate acceleration
        self.STAIR_ALLOW_YAW  = True     # Active yaw steering in STAIRS via Right Stick X (±0.50 rad/s)
        self.STAIR_HEADING_HOLD = True   # Closed-loop heading hold: actively locks yaw to prevent stair drift
        self.STAIR_KP_YAW     = 1.20     # Proportional gain for heading hold
        self.STAIR_MAX_YAW_CORR = 0.35   # Max corrective yaw rate (rad/s)
        self.stair_target_yaw = None     # Target heading yaw angle (rad)

        self.stairs_enter_t   = 0.0      # Timestamp when STAIRS mode was entered (for 0.3 s settle)
        self.gait_mismatch    = 0        # Debounce counter for telemetry gaitType mismatch in STAIRS
        self._last_gait_seen  = None     # Previous gait seen by [GAIT CHANGE] logger
        self.stair_rearm_pulse  = 0      # Number of frames to send gaitType=1 to create 1->3 rising edge
        self.stair_last_rearm_t = 0.0    # Cooldown timestamp for automatic rising edge re-arm

        # Pitch tilt (Right Stick Y in STAND mode)
        self.PITCH_MAX        = 0.35     # ±0.35 rad (±20°), official factory remote range
        self.ATTITUDE_DEADZONE= 0.08

        # Smooth velocity ramping filter (slew rate limiter)
        self.cmd_vx           = 0.0      # Filtered forward/backward velocity (m/s)
        self.cmd_vy           = 0.0      # Filtered lateral velocity (m/s)
        self.cmd_vyaw         = 0.0      # Filtered turn rate (rad/s)
        self.ACCEL_VX_UP      = 1.80     # Forward/backward acceleration (m/s^2)
        self.ACCEL_VX_DOWN    = 2.50     # Smooth deceleration to stop (m/s^2)
        self.ACCEL_VY         = 1.50     # Lateral strafe acceleration (m/s^2)
        self.ACCEL_VYAW       = 3.50     # Turn rate acceleration (rad/s^2)

        # Dynamic body height adjustment (D-Pad UP / DOWN continuous hold in STAND mode only)
        self.HEIGHT_MIN       = -0.10    # -10 cm crouch limit
        self.HEIGHT_MAX       = +0.08    # +8 cm tall stance limit
        self.HEIGHT_SPEED     = 0.12     # 12 cm/s continuous height ramp rate
        self.last_height_log  = 0.0      # Throttled height change logger timestamp
        self.dpad_adjusting   = False    # Flag to detect release of D-pad

        # Attitude targets (sent to robotControl each cycle)
        self.pitch       = 0.0
        self.roll        = 0.0
        self.yaw         = 0.0
        self.body_height = 0.0
        self.foot_raise  = 0.08

        # Telemetry exception guard flags
        self.telemetry_error_logged = False
        self._bms_err_logged        = False  # Logs BMS SOC telemetry read errors once

        # Battery telemetry & low-power safety threshold
        self.battery_soc      = 0        # Current BMS State of Charge (0-100%)
        self.low_bat_counter  = 0        # Debounce counter for low battery trigger
        self.LOW_BAT_LIMIT    = 20       # Auto-damp low battery threshold percentage (<= 20%)
        self.LOW_BAT_RESUME   = 22       # Minimum battery percentage to allow standing up (> 22%)

    # --------------------------------------------------------------------------
    # HARDWARE INITIALISATION — Bounded route check loop with shutdown_flag
    # --------------------------------------------------------------------------
    def init_robot(self, shutdown_flag):
        """
        Bounded route check loop (60 tries, 0.5 s apart) to verify local network
        stack route to ROBOT_IP:ROBOT_PORT before creating sdk.RobotInterface().
        Checks shutdown_flag[0] on each iteration.
        """
        logger.info("Checking route to robot internal network %s:%d ...", ROBOT_IP, ROBOT_PORT)
        for attempt in range(60):
            if shutdown_flag[0]:
                return False
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.settimeout(1.0)
                # Note: UDP connect() only validates local route table, not peer reachability
                s.connect((ROBOT_IP, ROBOT_PORT))
                s.close()
                break
            except Exception:
                time.sleep(0.5)
        else:
            logger.error("Could not establish route to %s:%d after 60 attempts.", ROBOT_IP, ROBOT_PORT)
            return False

        if shutdown_flag[0]:
            return False

        logger.info("Initialising Unitree High-Level UDP interface (port 8082) ...")
        self.robot = sdk.RobotInterface()
        logger.info("SDK interface ready. Zero MQTT.")
        return True

    # --------------------------------------------------------------------------
    # WAIT FOR CONTROLLER — Dynamic device scanning & robust open retry loop
    # --------------------------------------------------------------------------
    def wait_for_controller(self, shutdown_flag, keep_stance=False):
        """
        Scan for /dev/input/js* and block until a controller is available and opened.
        Retries open() with 0.2 s sleep on OSError to handle permission races on new nodes.
        If keep_stance=True, actively streams stance at 50 Hz according to current mode_state
        without abruptly changing gaits mid-motion.
        """
        last_log = 0.0
        while not shutdown_flag[0]:
            js_devices = sorted(glob.glob("/dev/input/js*"))
            if js_devices:
                self.js_path = js_devices[0]
                try:
                    self.js_fd = os.open(self.js_path, os.O_RDONLY | os.O_NONBLOCK)
                    logger.info("PS4 controller connected and opened at %s", self.js_path)
                    logger.info("NOTE: If D-Pad height adjustment does nothing, run 'jstest %s' to verify axis mapping (expected: Hat0X=axis 6, Hat0Y=axis 7).", self.js_path)
                    return True
                except OSError:
                    # Device node exists but permissions or driver initialization not ready yet
                    time.sleep(0.2)
            else:
                now = time.time()
                if now - last_log >= 2.0:
                    logger.info("Waiting for PS4 controller (/dev/input/js*) — press PS button ...")
                    last_log = now

            if keep_stance and self.robot:
                # Do not send mode 1 mid-gait. Maintain safe zero-velocity stance matching mode_state:
                if self.mode_state in ("PRONE", "DAMP"):
                    self.robot.robotControl(7, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                elif self.mode_state == "STAIRS":
                    self.robot.robotControl(2, 3, 0, self.STAIR_FOOT_RAISE, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                elif self.mode_state == "WALK":
                    self.robot.robotControl(2, 1, 0, self.WALK_FOOT_RAISE, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                else:  # STAND
                    self.robot.robotControl(1, 0, 0, 0.0, self.body_height, 0.0, self.pitch, 0.0, 0.0, 0.0, 0.0)
                self.robot.UDPSend()
                self.robot.UDPRecv()
            time.sleep(0.02)

        return False

    # --------------------------------------------------------------------------
    # HELPER: DRAIN STALE EVENTS WITHOUT TRIGGERING ACTIONS
    # --------------------------------------------------------------------------
    def _drain_state_only(self, axes, buttons):
        """
        Drain all pending joystick events accumulated during blocking sequences.
        Updates only the axes and buttons dictionaries (stripping JS_EVENT_INIT).
        Triggers NO actions, returning immediately on empty buffer or error.
        """
        if self.js_fd is None:
            return
        while True:
            try:
                chunk = os.read(self.js_fd, 8)
                if len(chunk) < 8:
                    break
                ts, val, ev_type, num = JS_EVENT_STRUCT.unpack(chunk)
                ev_clean = ev_type & ~JS_EVENT_INIT
                if ev_clean == JS_EVENT_AXIS:
                    axes[num] = max(-1.0, min(1.0, val / 32767.0))
                elif ev_clean == JS_EVENT_BUTTON:
                    buttons[num] = 1 if val else 0
            except (BlockingIOError, OSError):
                break
        self.dpad_adjusting = False

    # --------------------------------------------------------------------------
    # HELPER: ACCELERATION / DECELERATION SLEW RATE LIMITER
    # --------------------------------------------------------------------------
    @staticmethod
    def _ramp(current, target, rate_up, rate_down, dt):
        """
        Slew rate limiter with separate acceleration and deceleration rates.
        Prevents sudden velocity steps that disrupt the quadruped's dynamic balance.
        Snaps cleanly to zero if target is 0.0 and residual is below 0.005.
        """
        diff = target - current
        if abs(diff) < 1e-4:
            return target
        if target == 0.0 and abs(current) < 0.005:
            return 0.0
        is_decel = (abs(target) < abs(current)) or (current * target < 0)
        max_delta = (rate_down if is_decel else rate_up) * dt
        if diff > max_delta:
            return current + max_delta
        elif diff < -max_delta:
            return current - max_delta
        return target

    # --------------------------------------------------------------------------
    # HELPER: RAMP BODY HEIGHT TO NOMINAL (0.0 M) BEFORE ENTERING WALK / STAIRS
    # --------------------------------------------------------------------------
    def _ramp_height_to_zero(self, desc="Ramp body height to nominal"):
        """
        Ramp body_height to 0.0 over ~0.24 s in STAND mode before entering WALK or STAIRS.
        Avoids abrupt height step transitions when leaving STAND mode.
        """
        if abs(self.body_height) > 0.005:
            logger.info("  -> %s (current: %+.3f m) ...", desc, self.body_height)
            steps = 12  # 12 frames * 20 ms = ~0.24 s
            step_delta = self.body_height / steps
            for _ in range(steps):
                self.body_height -= step_delta
                self.robot.robotControl(
                    1, 0, 0, 0.0,
                    self.body_height, 0.0, self.pitch, 0.0,
                    0.0, 0.0, 0.0
                )
                self.robot.UDPSend()
                self.robot.UDPRecv()
                time.sleep(0.02)
            self.body_height = 0.0

    # --------------------------------------------------------------------------
    # HELPER: SMOOTHLY DECELERATE ACTIVE GAIT BEFORE LOCKING STANCE
    # --------------------------------------------------------------------------
    def smooth_stop_to_stand(self, desc="Settle to stand"):
        """
        Safely decelerate active gait to a standstill before locking stance.
        Ramps cmd_vx, cmd_vy, cmd_vyaw to 0.0 via _ramp() without changing gait:
        - If mode_state == 'STAIRS': gaitType 3, STAIR_FOOT_RAISE, up to 40 frames.
          Decel rates: STAIR_ACCEL_DOWN for vx and vy, STAIR_YAW_ACCEL for yaw.
        - Otherwise (WALK): gaitType 1, foot raise 0.06, up to 35 frames.
          Decel rates: ACCEL_VX_DOWN, ACCEL_VY, ACCEL_VYAW.
        Breaks early as soon as all commanded velocities reach zero (< 0.005 m/s).
        Then holds mode 1 (force stand) for 5 frames (0.1 s).
        Does NOT change mode_state inside this function. Never sends gaitType 1 while in STAIRS.
        """
        logger.info("  -> %s (active mode: %s) ...", desc, self.mode_state)
        if self.mode_state == "STAIRS":
            # Up to 40 frames of ramped deceleration in climb stair gait (gaitType 3)
            for _ in range(40):
                self.cmd_vx   = self._ramp(self.cmd_vx,   0.0, 1.0, self.STAIR_ACCEL_DOWN, LOOP_RATE)
                self.cmd_vy   = self._ramp(self.cmd_vy,   0.0, 1.0, self.STAIR_ACCEL_DOWN, LOOP_RATE)
                self.cmd_vyaw = self._ramp(self.cmd_vyaw, 0.0, 1.0, self.STAIR_YAW_ACCEL,  LOOP_RATE)
                self.robot.robotControl(2, 3, 0, self.STAIR_FOOT_RAISE, 0.0, 0.0, 0.0, 0.0, self.cmd_vx, self.cmd_vy, self.cmd_vyaw)
                self.robot.UDPSend()
                self.robot.UDPRecv()
                time.sleep(0.02)
                if abs(self.cmd_vx) < 0.005 and abs(self.cmd_vy) < 0.005 and abs(self.cmd_vyaw) < 0.005:
                    break
        else:
            # Up to 35 frames of ramped deceleration in trot gait (gaitType 1)
            for _ in range(35):
                self.cmd_vx   = self._ramp(self.cmd_vx,   0.0, 1.0, self.ACCEL_VX_DOWN, LOOP_RATE)
                self.cmd_vy   = self._ramp(self.cmd_vy,   0.0, 1.0, self.ACCEL_VY,      LOOP_RATE)
                self.cmd_vyaw = self._ramp(self.cmd_vyaw, 0.0, 1.0, self.ACCEL_VYAW,    LOOP_RATE)
                self.robot.robotControl(2, 1, 0, 0.06, 0.0, 0.0, 0.0, 0.0, self.cmd_vx, self.cmd_vy, self.cmd_vyaw)
                self.robot.UDPSend()
                self.robot.UDPRecv()
                time.sleep(0.02)
                if abs(self.cmd_vx) < 0.005 and abs(self.cmd_vy) < 0.005 and abs(self.cmd_vyaw) < 0.005:
                    break

        # 5 frames (0.1s) of force stand lock
        for _ in range(5):
            self.robot.robotControl(1, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.robot.UDPSend()
            self.robot.UDPRecv()
            time.sleep(0.02)
        self.cmd_vx = self.cmd_vy = self.cmd_vyaw = 0.0

    # --------------------------------------------------------------------------
    # HELPER: SEND A SPECIFIC MODE FOR N FRAMES AT 50 HZ
    # --------------------------------------------------------------------------
    def _send_mode_frames(self, mode, frames, desc=""):
        """
        Send a specific HighCmd mode for N frames at 50 Hz (20 ms interval).
        All kinematic offsets (height, roll, pitch, yaw, vx, vy, vyaw) are strictly 0.0.
        """
        if desc:
            logger.info("  -> Mode %d: %s (%d frames, %.1f s)", mode, desc, frames, frames * 0.02)
        for _ in range(frames):
            self.robot.robotControl(mode, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.robot.UDPSend()
            self.robot.UDPRecv()
            time.sleep(0.02)

    # --------------------------------------------------------------------------
    # INITIAL STATE SYNC — Read firmware state and match software state to it
    # --------------------------------------------------------------------------
    def sync_initial_state(self):
        """
        Sync software mode_state with actual robot firmware state on startup.
        Reads st.mode from robot telemetry and sets mode_state accordingly.
        Does NOT force DAMP if robot is already standing — avoids dropping a standing robot.
        NOTE: to test the standing branch of mode 0, the robot must be standing on the
        floor with a spotter (a held robot has no foot load and takes DAMP).
        """
        logger.info("Syncing initial state from robot firmware ...")

        # Prime the UDP channel — send neutral frames without changing robot state
        for _ in range(10):
            self.robot.UDPSend()
            self.robot.UDPRecv()
            time.sleep(0.02)

        try:
            st = self.robot.getState()
            fw_mode = st.mode
            logger.info("Firmware reports SDK mode=%d on startup.", fw_mode)

            if fw_mode in (1, 2, 6):
                # Robot is standing / walking — keep it in STAND, don't disrupt
                self.mode_state = "STAND"
                logger.info("Robot is STANDING (SDK mode=%d). mode_state = STAND.", fw_mode)
                # Reinforce mode 1 (force stand) to hold the posture
                self._send_mode_frames(1, 50, "Maintain standing posture on startup")

            elif fw_mode == 7:
                # Robot is in damping — stay in DAMP
                self.mode_state = "DAMP"
                logger.info("Robot is in DAMP (SDK mode=7). mode_state = DAMP.")
                self._send_mode_frames(7, 25, "Maintain DAMP on startup")

            elif fw_mode == 0:
                # Firmware mode 0 is the default stand / idle state.
                # NOTE: to test the standing branch, the robot must be standing on the floor
                # with a spotter (a held robot has no foot load, so it will take the DAMP fallback).
                # NOTE: I do not know whether st.bodyHeight reports an absolute height or an offset
                # near 0, so the 0.20 m and 0.12 m thresholds are unverified and must be set from
                # the logged [INIT SYNC] values.
                ff = list(st.footForce)
                body_h = float(st.bodyHeight)
                all_feet_loaded = all(f > 50 for f in ff)
                logger.info(
                    "[INIT SYNC] Firmware mode=0 (idle). Stance inspection: bodyHeight=%.3f m, "
                    "footForces=%s, all_loaded=%s",
                    body_h, ff, all_feet_loaded
                )
                if body_h > 0.20 and all_feet_loaded:
                    # Clearly standing: align posture in mode 6 (1.0 s) then lock force stand (mode 1)
                    logger.info("[INIT SYNC] Robot is clearly standing in mode 0. Transitioning 6 (1.0s) -> 1 (1.0s) to lock STAND.")
                    self._send_mode_frames(6, 50, "Mode 6 position posture alignment from mode 0")
                    self._send_mode_frames(1, 50, "Mode 1 force stand lock")
                    self.mode_state = "STAND"
                elif body_h < 0.12:
                    # Clearly lying down: safely set DAMP and send mode 7
                    logger.info("[INIT SYNC] Robot is clearly lying down (bodyHeight=%.3f m < 0.12 m). Setting DAMP.", body_h)
                    self.mode_state = "DAMP"
                    self._send_mode_frames(7, 25, "Maintain DAMP on startup")
                else:
                    # Unclear / ambiguous stance: default to DAMP for safety and log reason
                    logger.warning(
                        "[INIT SYNC] Mode 0 stance ambiguous (bodyHeight=%.3f m, footForces=%s). "
                        "Defaulting to DAMP for safety.", body_h, ff
                    )
                    self.mode_state = "DAMP"
                    self._send_mode_frames(7, 25, "Safe default DAMP (unclear mode 0 stance)")

            else:
                # Other ambiguous mode (e.g. mode 5, 8 …) — safe default: DAMP
                self.mode_state = "DAMP"
                logger.info(
                    "Firmware mode=%d is ambiguous. Defaulting to DAMP for safety.", fw_mode)
                self._send_mode_frames(7, 25, "Safe default DAMP")

        except Exception as ex:
            logger.warning(
                "Could not read initial robot state: %s. Defaulting to DAMP.", ex)
            self.mode_state = "DAMP"
            self._send_mode_frames(7, 25, "Safe default DAMP (exception fallback)")

        logger.info(
            "Initial state sync complete. mode_state = %s. "
            "Press L2 + SQUARE to stand up (if in DAMP).",
            self.mode_state)

    # --------------------------------------------------------------------------
    # OFFICIAL SIT DOWN — 1 (0.5s) -> 6 (1.0s) -> 5 (2.0s) -> 7 (1.0s) -> DAMP
    # --------------------------------------------------------------------------
    def smooth_sit_down(self):
        """
        Official Unitree Sit-Down sequence:
        Mode 1 (0.5 s, force-stand) -> Mode 6 (1.0 s, position align) ->
        Mode 5 (2.0 s, position stand down) -> Mode 7 (1.0 s, damping relax).
        Ends with self.mode_state = "DAMP". Mode 5 is transitional and not held.
        """
        logger.info("Starting official Sit-Down sequence (1 -> 6 -> 5 -> 7) ...")
        self.pitch = self.roll = self.yaw = 0.0
        self.body_height = 0.0
        self.cmd_vx = self.cmd_vy = self.cmd_vyaw = 0.0

        # 1. Force stand to ensure zero motion and steady stance (0.5 s = 25 frames)
        self._send_mode_frames(1, 25,  "Force Stand settle")
        # 2. Position Stand Up to align posture (1.0 s = 50 frames)
        self._send_mode_frames(6, 50,  "Position Stand Up posture alignment")
        # 3. Position Stand Down kinematic descent (2.0 s = 100 frames)
        self._send_mode_frames(5, 100, "Position Stand Down kinematic descent")
        # 4. Damping motor relax (1.0 s = 50 frames)
        self._send_mode_frames(7, 50,  "Damping motor relax")

        self.mode_state = "DAMP"
        logger.info("Sit-Down complete. Robot resting flat in DAMP (mode 7).")

    # --------------------------------------------------------------------------
    # OFFICIAL STAND UP — 7 (0.5s) -> 5 (1.0s) -> 6 (2.0s) -> 1 (1.0s) -> STAND
    # --------------------------------------------------------------------------
    def smooth_stand_up(self):
        """
        Official Unitree Stand-Up sequence:
        Mode 7 (0.5 s, damping check) -> Mode 5 (1.0 s, foot positioning) ->
        Mode 6 (2.0 s, position stand up) -> Mode 1 (1.0 s, force-stand lock).
        Ends with self.mode_state = "STAND".
        """
        logger.info("Starting official Stand-Up sequence (7 -> 5 -> 6 -> 1) ...")
        self.pitch = self.roll = self.yaw = 0.0
        self.body_height = 0.0
        self.cmd_vx = self.cmd_vy = self.cmd_vyaw = 0.0

        # 1. Ensure damping state (0.5 s = 25 frames)
        self._send_mode_frames(7, 25,  "Damping pre-check")
        # 2. Mode 5 pre-stand foot positioning (1.0 s = 50 frames)
        self._send_mode_frames(5, 50,  "Stand Down foot positioning")
        # 3. Mode 6 position stand up ascent (2.0 s = 100 frames)
        self._send_mode_frames(6, 100, "Position Stand Up ascent")
        # 4. Mode 1 force-stand lock (1.0 s = 50 frames)
        self._send_mode_frames(1, 50,  "Force Stand lock")

        self.mode_state = "STAND"
        logger.info("Stand-Up complete. Robot locked firmly in STAND (mode 1).")

    # --------------------------------------------------------------------------
    # MAIN CONTROL LOOP
    # --------------------------------------------------------------------------
    def run(self):
        """Main 50 Hz direct UDP teleoperation loop."""

        # ---- Signal handling ------------------------------------------------
        shutdown_flag = [False]

        def _sig_handler(sig, frame):
            logger.info("Signal %d received — shutting down ...", sig)
            shutdown_flag[0] = True

        signal.signal(signal.SIGINT,  _sig_handler)
        signal.signal(signal.SIGTERM, _sig_handler)

        # ---- Startup sequence -----------------------------------------------
        if not self.init_robot(shutdown_flag):
            return
        if not self.wait_for_controller(shutdown_flag):
            return
        self.sync_initial_state()

        # ---- Input state buffers --------------------------------------------
        axes    = {i: 0.0 for i in range(8)}
        buttons = {i: 0   for i in range(13)}

        # ---- Telemetry - velocity outputs initialised to zero --------------
        vx = vy = vyaw = 0.0
        st_live = None

        last_log_t = 0.0
        logger.info("50 Hz teleoperation loop is LIVE. Controller active.")

        # ======================================================================
        # LOOP START
        # ======================================================================
        try:
            while not shutdown_flag[0]:
                cycle_start = time.time()

                # ==============================================================
                # STEP 1: READ ALL PENDING JOYSTICK EVENTS (NON-BLOCKING)
                # ==============================================================
                while True:
                    try:
                        chunk = os.read(self.js_fd, 8)
                        if len(chunk) < 8:
                            break
                        ts, val, ev_type, num = JS_EVENT_STRUCT.unpack(chunk)

                        # Axis movement
                        if ev_type & JS_EVENT_AXIS:
                            # Normalise to -1.0 .. +1.0
                            norm_val = max(-1.0, min(1.0, val / 32767.0))
                            axes[num] = norm_val

                        # Button event
                        elif ev_type & JS_EVENT_BUTTON:
                            pressed = 1 if val else 0
                            buttons[num] = pressed

                            # Ignore synthetic driver init events (JS_EVENT_INIT) for action triggering
                            if pressed and not (ev_type & JS_EVENT_INIT):
                                # Determine chord state: is L2 held?
                                l2_held = (buttons.get(BTN_L2, 0) == 1 or
                                           axes.get(AXIS_L2, -1.0) > 0.5)

                                # ==============================================
                                # L2 CHORD COMBOS — primary mode switches
                                # ==============================================
                                if l2_held:
                                    # L2 + TRIANGLE  →  WALK (allowed from STAND or STAIRS)
                                    if num == BTN_TRIANGLE:
                                        if self.mode_state in ("STAND", "STAIRS"):
                                            if self.mode_state == "STAIRS":
                                                self.smooth_stop_to_stand("Settle to Stand before Walk mode")
                                                self._drain_state_only(axes, buttons)
                                            elif self.mode_state == "STAND":
                                                self._ramp_height_to_zero("Ramp height to nominal before Walk mode")
                                                self._drain_state_only(axes, buttons)
                                            self.mode_state  = "WALK"
                                            self.foot_raise  = self.WALK_FOOT_RAISE
                                            self.pitch = self.roll = self.yaw = 0.0
                                            self.body_height = 0.0
                                            self.cmd_vx = self.cmd_vy = self.cmd_vyaw = 0.0
                                            logger.info("[MODE] >>> WALKING MODE (8 cm trot, gaitType 1) <<<")
                                        elif self.mode_state in ("PRONE", "DAMP"):
                                            logger.warning(
                                                "[BLOCKED] Cannot enter WALK from %s — "
                                                "press L2 + SQUARE to stand up first.",
                                                self.mode_state)

                                    # L2 + SQUARE  →  STAND / STAND UP (the ONLY button to exit DAMP)
                                    elif num == BTN_SQUARE:
                                        # Check if robot needs to execute the stand up sequence:
                                        # Either software mode is DAMP/PRONE, OR robot firmware telemetry reports SDK mode 7 (damped/limp)
                                        is_damped = (self.mode_state in ("PRONE", "DAMP"))
                                        if not is_damped and self.robot:
                                            try:
                                                st_chk = self.robot.getState()
                                                if st_chk.mode == 7:
                                                    is_damped = True
                                                    logger.info("[STAND SYNC] Firmware telemetry indicates SDK mode 7 (DAMP).")
                                            except Exception:
                                                pass

                                        if is_damped:
                                            # Low-battery lockout: ONLY blocks standing up from the floor!
                                            if 1 <= self.battery_soc <= self.LOW_BAT_RESUME:
                                                logger.warning(
                                                    "[BLOCKED] Cannot stand up from floor: Battery low (%d%% <= %d%% required)! "
                                                    "Please charge the robot before operating.",
                                                    self.battery_soc, self.LOW_BAT_RESUME)
                                                continue

                                            logger.info("[STAND UP] Executing smooth stand up sequence from DAMP ...")
                                            self.smooth_stand_up()
                                            self._drain_state_only(axes, buttons)
                                            self.pitch = self.roll = self.yaw = 0.0
                                            self.body_height = 0.0
                                            logger.info("[MODE] >>> STANDING / LOCK MODE <<<")
                                        else:
                                            # Not damped: transition from WALK or STAIRS to STAND
                                            if self.mode_state in ("WALK", "STAIRS"):
                                                self.smooth_stop_to_stand("Settle to Stand from " + self.mode_state)
                                                self._drain_state_only(axes, buttons)
                                            self.mode_state  = "STAND"
                                            self.pitch = self.roll = self.yaw = 0.0
                                            self.body_height = 0.0
                                            self.cmd_vx = self.cmd_vy = self.cmd_vyaw = 0.0
                                            logger.info("[MODE] >>> STANDING / LOCK MODE <<<")

                                    # L2 + CROSS  →  STAIRS (allowed from STAND or WALK)
                                    elif num == BTN_CROSS:
                                        if self.mode_state in ("STAND", "WALK"):
                                            if self.mode_state == "WALK":
                                                self.smooth_stop_to_stand("Settle to Stand before Stair mode")
                                                self._drain_state_only(axes, buttons)
                                            elif self.mode_state == "STAND":
                                                self._ramp_height_to_zero("Ramp height to nominal before Stair mode")
                                                self._drain_state_only(axes, buttons)
                                            self.mode_state  = "STAIRS"
                                            self.stairs_enter_t = time.time()
                                            self.gait_mismatch  = 0
                                            self.stair_rearm_pulse = 0
                                            self.stair_last_rearm_t = 0.0
                                            self.foot_raise  = self.STAIR_FOOT_RAISE
                                            self.pitch = self.roll = self.yaw = 0.0
                                            self.body_height = 0.0
                                            self.cmd_vx = self.cmd_vy = self.cmd_vyaw = 0.0
                                            logger.info("[MODE] >>> OFFICIAL STAIRS MODE (gaitType 3, climb stair) <<<")
                                        elif self.mode_state in ("PRONE", "DAMP"):
                                            logger.warning(
                                                "[BLOCKED] Cannot enter STAIRS from %s — "
                                                "press L2 + SQUARE to stand up first.",
                                                self.mode_state)

                                    # L2 + CIRCLE  →  SIT DOWN (allowed from STAND only; ends in DAMP)
                                    elif num == BTN_CIRCLE:
                                        if self.mode_state == "STAND":
                                            logger.info("[CMD] L2 + CIRCLE: Executing Sit-Down sequence ...")
                                            self.smooth_sit_down()
                                            self._drain_state_only(axes, buttons)
                                            self.pitch = self.roll = self.yaw = 0.0
                                            logger.info("[MODE] >>> DAMP / RESTING ON FLOOR <<<")
                                        elif self.mode_state in ("PRONE", "DAMP"):
                                            logger.info("[DAMP] Robot is already lying down in DAMP.")
                                        else:
                                            logger.warning(
                                                "[BLOCKED] Cannot sit down from %s mode — "
                                                "switch to STAND (L2 + SQUARE) first.",
                                                self.mode_state)

                    except BlockingIOError:
                        # No more events in the queue — normal, continue loop
                        break

                    except OSError:
                        # Controller disconnected
                        logger.warning(
                            "Controller disconnected! "
                            "Freezing motion and holding posture at 50 Hz ...")
                        try:
                            os.close(self.js_fd)
                        except Exception:
                            pass
                        # Clear all inputs immediately — prevent runaway velocity
                        axes    = {i: 0.0 for i in range(8)}
                        buttons = {i: 0   for i in range(13)}
                        self.pitch = self.roll = self.yaw = 0.0
                        self.cmd_vx = self.cmd_vy = self.cmd_vyaw = 0.0
                        # Hold posture at 50 Hz until controller comes back
                        if not self.wait_for_controller(shutdown_flag, keep_stance=True):
                            break
                        # Sync mode_state with physical robot state on reconnect
                        try:
                            st = self.robot.getState()
                            if st.mode == 7:
                                self.mode_state = "DAMP"
                                logger.info("[SYNC] Robot is physically in DAMP (SDK mode 7). mode_state set to DAMP.")
                            elif st.mode in (0, 1, 2, 6):
                                # Only overwrite mode_state to STAND if not currently in WALK or STAIRS
                                if self.mode_state not in ("WALK", "STAIRS"):
                                    self.mode_state = "STAND"
                                    logger.info("[SYNC] Robot is physically standing (SDK mode %d). mode_state set to STAND.", st.mode)
                                else:
                                    logger.info("[SYNC] Robot reconnected in %s mode (SDK mode %d). Maintaining %s posture.",
                                                self.mode_state, st.mode, self.mode_state)
                        except Exception:
                            pass
                        self._drain_state_only(axes, buttons)
                        logger.info(
                            "Controller reconnected. Resuming in %s mode.", self.mode_state)
                        break

                # ==============================================================
                # STEP 2: READ STICK VALUES & CONTINUOUS HEIGHT ADJUSTMENT
                # ==============================================================
                raw_vx    = -axes.get(AXIS_LS_Y, 0.0)  # push forward → positive VX
                raw_vy    = -axes.get(AXIS_LS_X, 0.0)  # push left    → positive VY
                raw_vyaw  = -axes.get(AXIS_RS_X, 0.0)  # push left    → turn left (+)
                raw_pitch =  axes.get(AXIS_RS_Y, 0.0)  # push forward → tilt down
                dpad_y    =  axes.get(AXIS_DPAD_Y, 0.0) # Hat0Y: Up=-1.0, Down=+1.0

                # Continuous body height adjust on held D-Pad UP / DOWN (STAND mode only)
                if self.mode_state == "STAND":
                    if dpad_y < -0.5:
                        # D-Pad UP held: ramp height up continuously (max +0.08 m)
                        self.body_height = round(
                            min(self.HEIGHT_MAX, self.body_height + self.HEIGHT_SPEED * 0.02), 4)
                        self.dpad_adjusting = True
                        now_h = time.time()
                        if now_h - self.last_height_log >= 0.3:
                            logger.info("[HEIGHT] Holding D-Pad UP -> body_height = %+.3f m%s",
                                        self.body_height,
                                        " (MAX LIMIT)" if self.body_height >= self.HEIGHT_MAX else "")
                            self.last_height_log = now_h

                    elif dpad_y > 0.5:
                        # D-Pad DOWN held: ramp height down continuously (min -0.10 m)
                        self.body_height = round(
                            max(self.HEIGHT_MIN, self.body_height - self.HEIGHT_SPEED * 0.02), 4)
                        self.dpad_adjusting = True
                        now_h = time.time()
                        if now_h - self.last_height_log >= 0.3:
                            logger.info("[HEIGHT] Holding D-Pad DOWN -> body_height = %+.3f m%s",
                                        self.body_height,
                                        " (MIN LIMIT)" if self.body_height <= self.HEIGHT_MIN else "")
                            self.last_height_log = now_h

                    else:
                        if self.dpad_adjusting:
                            logger.info("[HEIGHT] D-Pad released -> settled body_height = %+.3f m", self.body_height)
                            self.dpad_adjusting = False
                            self.last_height_log = 0.0
                else:
                    # In WALK, STAIRS, or DAMP mode, height adjust is locked to nominal
                    if abs(dpad_y) > 0.5 and not self.dpad_adjusting:
                        logger.info("[HEIGHT] Height adjust is only allowed in STAND mode (current: %s).", self.mode_state)
                        self.dpad_adjusting = True
                    elif abs(dpad_y) <= 0.5 and self.dpad_adjusting:
                        self.dpad_adjusting = False

                # ==============================================================
                # STEP 3: EXECUTE ACTIVE MODE
                # ==============================================================

                # --------------------------------------------------------------
                # STAND — joints locked, pitch tilt on Right Stick Y only
                # --------------------------------------------------------------
                if self.mode_state == "STAND":
                    self.cmd_vx = self.cmd_vy = self.cmd_vyaw = 0.0
                    vx = vy = vyaw = 0.0
                    self.roll = self.yaw = 0.0

                    # Pitch: deadzone → normalise → scale to ±PITCH_MAX
                    if abs(raw_pitch) > self.ATTITUDE_DEADZONE:
                        norm_pitch = math.copysign(
                            (abs(raw_pitch) - self.ATTITUDE_DEADZONE)
                            / (1.0 - self.ATTITUDE_DEADZONE),
                            raw_pitch
                        )
                        target_pitch = norm_pitch * self.PITCH_MAX
                    else:
                        target_pitch = 0.0  # stick released → spring back to level

                    # Smooth spring-back (25% step per cycle @ 50 Hz ≈ 0.5 s settle)
                    self.pitch += (target_pitch - self.pitch) * 0.25
                    if target_pitch == 0.0 and abs(self.pitch) < 0.003:
                        self.pitch = 0.0  # snap to exactly 0 to avoid micro-drift

                    self.robot.robotControl(
                        1, 0, 0, 0.0,
                        self.body_height, 0.0, self.pitch, 0.0,
                        0.0, 0.0, 0.0
                    )

                # --------------------------------------------------------------
                # WALK — trot gait, 2-D radial deadzone, smooth slew rate ramp
                # --------------------------------------------------------------
                elif self.mode_state == "WALK":
                    self.pitch = self.roll = self.yaw = 0.0

                    # 2-D radial deadzone for Left Stick (avoids diagonal snap)
                    stick_mag = math.hypot(raw_vx, raw_vy)
                    if stick_mag > self.STICK_DEADZONE:
                        norm_mag = min(1.0, (stick_mag - self.STICK_DEADZONE)
                                            / (1.0 - self.STICK_DEADZONE))
                        vx_n = (raw_vx / stick_mag) * norm_mag
                        vy_n = (raw_vy / stick_mag) * norm_mag
                    else:
                        vx_n = vy_n = 0.0

                    # 1-D deadzone for Right Stick X (yaw)
                    yaw_mag = abs(raw_vyaw)
                    if yaw_mag > self.STICK_DEADZONE:
                        vyaw_n = math.copysign(
                            min(1.0, (yaw_mag - self.STICK_DEADZONE)
                                     / (1.0 - self.STICK_DEADZONE)),
                            raw_vyaw
                        )
                    else:
                        vyaw_n = 0.0

                    target_vx   = (vx_n * self.WALK_VX_FWD) if vx_n >= 0.0 else (vx_n * self.WALK_VX_BWD)
                    target_vy   = vy_n   * self.WALK_VY
                    target_vyaw = vyaw_n * self.WALK_VYAW

                    # Apply smooth acceleration/deceleration ramping to protect balance
                    self.cmd_vx   = self._ramp(self.cmd_vx,   target_vx,   self.ACCEL_VX_UP, self.ACCEL_VX_DOWN, LOOP_RATE)
                    self.cmd_vy   = self._ramp(self.cmd_vy,   target_vy,   self.ACCEL_VY,    self.ACCEL_VY,      LOOP_RATE)
                    self.cmd_vyaw = self._ramp(self.cmd_vyaw, target_vyaw, self.ACCEL_VYAW,  self.ACCEL_VYAW,    LOOP_RATE)

                    vx   = self.cmd_vx
                    vy   = self.cmd_vy
                    vyaw = self.cmd_vyaw

                    # Always send mode 2, gaitType 1 — even when velocity ramps to 0.0.
                    # Never abruptly switch to mode 1 on stick release: mode 2 with vx=0
                    # naturally lets the Go1 MPC active trot controller step to a
                    # perfectly balanced stop without stumbling or pitching forward.
                    self.robot.robotControl(
                        2, 1, 0, self.WALK_FOOT_RAISE,
                        0.0, 0.0, 0.0, 0.0,
                        vx, vy, vyaw
                    )

                # --------------------------------------------------------------
                # STAIRS — Unitree Climb Stair Mode (gaitType = 3)
                # Locomotion architecture:
                # - Mode 2, gaitType 3 with STAIR_FOOT_RAISE (0.08 m clearance delta).
                # - Continuous Mode 2 streaming: When Left Stick is released, velocity
                #   ramps smoothly to 0.0 m/s without cutting to Mode 1. This allows the
                #   underlying ConvexMPCLocomotionStairs controller to complete its gait
                #   cycle naturally and plant all 4 feet firmly in stance (preventing mid-air freezes).
                # - Closed-loop Heading Hold: Automatically latches target yaw angle on
                #   forward/backward motion and injects proportional yaw rate corrections (Kp=1.20)
                #   to prevent stair climbing drift. Right Stick X provides instant manual steering.
                # --------------------------------------------------------------
                elif self.mode_state == "STAIRS":
                    self.pitch = self.roll = self.yaw = 0.0

                    # Forward / backward (Left Stick Y): 1-D STAIR_DEADZONE (0.20)
                    if abs(raw_vx) > self.STAIR_DEADZONE:
                        vx_n = math.copysign(
                            (abs(raw_vx) - self.STAIR_DEADZONE) / (1.0 - self.STAIR_DEADZONE),
                            raw_vx
                        )
                        target_vx = (vx_n * self.STAIR_VX_FWD) if vx_n >= 0.0 else (vx_n * self.STAIR_VX_BWD)
                    else:
                        target_vx = 0.0

                    # Yaw steering & Closed-Loop Heading Hold
                    if self.STAIR_ALLOW_YAW and abs(raw_vyaw) > self.STAIR_DEADZONE:
                        # Manual steering has priority: clear heading lock
                        self.stair_target_yaw = None
                        vyaw_n = math.copysign(
                            min(1.0, (abs(raw_vyaw) - self.STAIR_DEADZONE)
                                     / (1.0 - self.STAIR_DEADZONE)),
                            raw_vyaw
                        )
                        target_vyaw = vyaw_n * self.STAIR_VYAW
                    elif self.STAIR_HEADING_HOLD and abs(self.cmd_vx) > 0.01 and st_live is not None:
                        # Closed-Loop Heading Hold: lock heading on forward/backward motion to eliminate drift!
                        curr_yaw = st_live.imu.rpy[2]
                        if self.stair_target_yaw is None:
                            self.stair_target_yaw = curr_yaw
                        # Shortest-angle difference in [-pi, pi]
                        yaw_diff = math.atan2(math.sin(self.stair_target_yaw - curr_yaw),
                                              math.cos(self.stair_target_yaw - curr_yaw))
                        target_vyaw = max(-self.STAIR_MAX_YAW_CORR, min(self.STAIR_MAX_YAW_CORR, self.STAIR_KP_YAW * yaw_diff))
                    else:
                        # Stationary or stick released: reset target yaw
                        self.stair_target_yaw = None
                        target_vyaw = 0.0

                    vy = 0.0  # Locked to 0.0 for official gaitType 3 crawl compatibility

                    # Slew rate filtering with stair-specific acceleration and deceleration
                    self.cmd_vx   = self._ramp(self.cmd_vx,   target_vx,   self.STAIR_ACCEL_UP,  self.STAIR_ACCEL_DOWN, LOOP_RATE)
                    self.cmd_vy   = 0.0
                    self.cmd_vyaw = self._ramp(self.cmd_vyaw, target_vyaw, self.STAIR_YAW_ACCEL, self.STAIR_YAW_ACCEL,  LOOP_RATE)

                    vx   = self.cmd_vx
                    vy   = 0.0
                    vyaw = self.cmd_vyaw

                    # In-motion stumble recovery: if firmware slips into trot while stick is held forward
                    now_t = time.time()
                    if (self.stair_rearm_pulse == 0 and
                        (now_t - self.stair_last_rearm_t) >= 0.4 and
                        st_live is not None and
                        st_live.gaitType != 3 and
                        abs(self.cmd_vx) > 0.01):
                        if abs(st_live.imu.rpy[0]) < 0.20 and abs(st_live.imu.rpy[1]) < 0.25:
                            self.stair_rearm_pulse = 2
                            self.stair_last_rearm_t = now_t
                            logger.info("[STAIR RE-ARM] In-motion recovery: pulsing rising edge for gaitType %d -> 3.", st_live.gaitType)

                    if self.stair_rearm_pulse > 0:
                        stair_gait_cmd = 1
                        self.stair_rearm_pulse -= 1
                    else:
                        stair_gait_cmd = 3

                    # Always send Mode 2, gaitType 3 continuously while in STAIRS mode.
                    # Exactly like WALK mode: sending Mode 2 with vx=0, vyaw=0 lets the Unitree
                    # ConvexMPCLocomotionStairs complete its step cycle and settle into a rock-solid,
                    # level 4-leg stance naturally without freezing swing legs mid-air or dropping height!
                    self.robot.robotControl(
                        2, stair_gait_cmd, 0, self.STAIR_FOOT_RAISE,
                        0.0, 0.0, 0.0, 0.0,
                        vx, 0.0, vyaw
                    )

                # --------------------------------------------------------------
                # DAMP / PRONE — resting on floor, send ONLY mode 7 every cycle
                # --------------------------------------------------------------
                elif self.mode_state in ("PRONE", "DAMP"):
                    self.cmd_vx = self.cmd_vy = self.cmd_vyaw = 0.0
                    vx = vy = vyaw = 0.0
                    self.body_height = 0.0
                    self.pitch = self.roll = self.yaw = 0.0
                    # Continuously send ONLY mode 7 at 50 Hz to keep motors relaxed
                    # and feed the UDP watchdog. Never send mode 1 or mode 5 here.
                    self.robot.robotControl(
                        7, 0, 0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0
                    )

                # Flush the UDP packet for this cycle
                self.robot.UDPSend()
                self.robot.UDPRecv()

                # Read real-time state from robot once per cycle
                try:
                    st_live = self.robot.getState()
                except Exception as ex:
                    st_live = None
                    if not getattr(self, "_state_err_logged", False):
                        logger.warning("getState() error: %s", ex)
                        self._state_err_logged = True

                # [GAIT CHANGE] Diagnostic Logger: fires on every change of st_live.gaitType
                if st_live is not None:
                    curr_gait = st_live.gaitType
                    last_gait = getattr(self, "_last_gait_seen", None)
                    if last_gait is not None and curr_gait != last_gait:
                        ff_int = [int(f) for f in st_live.footForce]
                        logger.warning(
                            "[GAIT CHANGE] gaitType changed %s -> %d | mode_state=%s fw_mode=%d | "
                            "cmd_vx=%.2f cmd_vyaw=%.2f | IMU roll=%.3f pitch=%.3f | footForce=%s",
                            last_gait, curr_gait, self.mode_state, st_live.mode,
                            self.cmd_vx, self.cmd_vyaw,
                            st_live.imu.rpy[0], st_live.imu.rpy[1],
                            ff_int
                        )
                    self._last_gait_seen = curr_gait

                # Update BMS battery state
                if st_live is not None:
                    try:
                        soc_raw = int(st_live.bms.SOC)
                        if 1 <= soc_raw <= 100:
                            if self.battery_soc == 0:
                                logger.info("[BMS] Battery telemetry connected: %d%%", soc_raw)
                            self.battery_soc = soc_raw
                    except Exception as ex:
                        if not getattr(self, "_bms_err_logged", False):
                            logger.warning("[BMS] Error reading BMS SOC telemetry: %s", ex)
                            self._bms_err_logged = True

                # Gait mismatch diagnostic in STAIRS mode:
                # Evaluates during active motion commands (> 0.3 s after stair entry).
                is_commanding_stair_motion = (abs(self.cmd_vx) > 0.03 or abs(self.cmd_vyaw) > 0.05)
                if self.mode_state == "STAIRS" and (time.time() - self.stairs_enter_t >= 0.3) and is_commanding_stair_motion and st_live is not None:
                    if st_live.mode == 2 and st_live.gaitType != 3:
                        self.gait_mismatch += 1
                        if self.gait_mismatch == 15:  # ~0.3s of continuous active motion with mismatch
                            logger.warning(
                                "[GAIT MISMATCH DIAGNOSTIC] Commanded mode 2 + gaitType 3 (STAIRS), but firmware reports "
                                "gaitType=%d during active motion! (mode=%d, cmd_vx=%.2f, cmd_vyaw=%.2f, IMU_roll=%.3f, IMU_pitch=%.3f). "
                                "Cause unknown: could be firmware fallback, terrain, or telemetry reporting.",
                                st_live.gaitType, st_live.mode, self.cmd_vx, self.cmd_vyaw,
                                st_live.imu.rpy[0], st_live.imu.rpy[1]
                            )
                    else:
                        self.gait_mismatch = 0
                else:
                    self.gait_mismatch = 0

                # Low-Battery Safety Monitor: auto-sit down into DAMP if battery <= 20%
                if 1 <= self.battery_soc <= self.LOW_BAT_LIMIT:
                    self.low_bat_counter += 1
                    # Debounce: require 50 consecutive cycles (1.0 s) to trigger
                    if self.low_bat_counter >= 50:
                        if self.mode_state not in ("PRONE", "DAMP"):
                            logger.warning(
                                "[LOW BATTERY CRITICAL] Battery level at %d%% (<= %d%%)! "
                                "Executing automatic safe sit-down into DAMP mode to protect cells & hardware ...",
                                self.battery_soc, self.LOW_BAT_LIMIT)
                            if self.mode_state == "STAIRS":
                                logger.warning(
                                    "[LOW BATTERY STAIRS WARNING] Robot is in STAIRS mode! "
                                    "Bringing to controlled stair stop before sit-down.")
                            if self.mode_state in ("WALK", "STAIRS"):
                                self.smooth_stop_to_stand("Low battery stop before sit-down")
                                self._drain_state_only(axes, buttons)
                            self.smooth_sit_down()
                            self._drain_state_only(axes, buttons)
                            self.pitch = self.roll = self.yaw = 0.0
                            self.body_height = 0.0
                            logger.warning(
                                "[LOW BATTERY CRITICAL] Robot is resting flat in DAMP mode. "
                                "Recharge required before operating.")
                else:
                    self.low_bat_counter = 0

                # ==============================================================
                # STEP 4: TELEMETRY LOG  (every 1 second)
                # ==============================================================
                now = time.time()
                if now - last_log_t >= 1.0:
                    last_log_t = now
                    if st_live is not None:
                        ff       = list(st_live.footForce)    # [FL, FR, RL, RR]  N
                        vel      = list(st_live.velocity)     # [vx, vy, vz]      m/s
                        imu_rpy  = list(st_live.imu.rpy)      # [roll, pitch, yaw] rad
                        body_h   = st_live.bodyHeight         # m
                        sdk_mode = st_live.mode
                        gait     = st_live.gaitType
                    else:
                        ff       = [0, 0, 0, 0]
                        vel      = imu_rpy = [0.0, 0.0, 0.0]
                        body_h   = 0.0
                        sdk_mode = gait = 0

                    contact = "".join("X" if f > 50 else "." for f in ff)

                    # Visual battery bar graphic (e.g. [████████░░]  85%)
                    if self.battery_soc > 0:
                        bars = int(round(self.battery_soc / 10.0))
                        bat_bar = "█" * min(10, bars) + "░" * max(0, 10 - bars)
                        bat_disp = "%3d%%" % self.battery_soc
                    else:
                        bat_bar = "░" * 10
                        bat_disp = " --%"

                    logger.info(
                        "\n"
                        "  ╔════════════════════════════════════════════════════╗\n"
                        "  ║  MODE    : %-8s  SDK mode=%-2d  gait=%-2d          ║\n"
                        "  ║  BATTERY : %s  [%s]                     ║\n"
                        "  ╠════════════════════════════════════════════════════╣\n"
                        "  ║  CMD VEL : VX=%+.2f  VY=%+.2f  YAW=%+.2f           ║\n"
                        "  ║  ACT VEL : VX=%+.2f  VY=%+.2f  VZ=%+.2f (m/s)      ║\n"
                        "  ╠════════════════════════════════════════════════════╣\n"
                        "  ║  PITCH   : cmd=%+.3f rad                          ║\n"
                        "  ║  IMU     : R=%+.3f  P=%+.3f  Y=%+.3f (rad)      ║\n"
                        "  ║  HEIGHT  : cmd=%+.3f m   actual=%+.3f m          ║\n"
                        "  ╠════════════════════════════════════════════════════╣\n"
                        "  ║  FEET    : FL=%4d FR=%4d RL=%4d RR=%4d N [%s]║\n"
                        "  ╚════════════════════════════════════════════════════╝",
                        self.mode_state, sdk_mode, gait,
                        bat_disp, bat_bar,
                        vx, vy, vyaw,
                        vel[0], vel[1], vel[2],
                        self.pitch,
                        imu_rpy[0], imu_rpy[1], imu_rpy[2],
                        self.body_height, body_h,
                        ff[0], ff[1], ff[2], ff[3], contact
                    )

                # ==============================================================
                # STEP 5: SLEEP TO ENFORCE STRICT 50 Hz
                # ==============================================================
                elapsed = time.time() - cycle_start
                if elapsed < LOOP_RATE:
                    time.sleep(LOOP_RATE - elapsed)

        except KeyboardInterrupt:
            # KeyboardInterrupt is handled via the SIGINT signal handler (_sig_handler),
            # kept here as an explicit fallback so unexpected interrupts cleanly terminate.
            logger.info("KeyboardInterrupt — shutting down.")

        finally:
            # ------------------------------------------------------------------
            # SAFE SHUTDOWN: preserve stance safely before exiting
            # NOTE: When the script exits while in DAMP, the firmware may fall back
            # to mode 0 (idle) after HighCmd packets cease. This behavior must be
            # tested once with the robot held.
            # ------------------------------------------------------------------
            logger.info("Safe shutdown: preserving stance safely before exiting ...")
            if self.robot is not None and self.mode_state in ("WALK", "STAIRS"):
                try:
                    self.smooth_stop_to_stand("Shutdown stop from " + self.mode_state)
                except Exception as ex:
                    logger.warning("Error during shutdown stop sequence: %s", ex)
                self.mode_state = "STAND"

            if self.robot:
                mode_to_send = 7 if self.mode_state in ("PRONE", "DAMP") else 1
                for _ in range(25):
                    self.robot.robotControl(mode_to_send, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                    self.robot.UDPSend()
                    self.robot.UDPRecv()
                    time.sleep(0.02)
            try:
                if self.js_fd:
                    os.close(self.js_fd)
            except Exception:
                pass
            logger.info("Shutdown complete.")


# ==============================================================================
# ENTRY POINT
# ==============================================================================
if __name__ == "__main__":
    controller = Go1PS4Controller()
    controller.run()
