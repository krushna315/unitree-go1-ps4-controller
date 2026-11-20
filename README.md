# Unitree Go1 Air — PS4 DualShock 4 Controller Teleoperation System

A production-grade, high-performance teleoperation daemon for the **Unitree Go1 Air** quadruped robot using a Sony PS4 DualShock 4 wireless controller. 

Built on a direct **High-Level UDP SDK architecture (50 Hz, port 8082)**, this system operates with zero ROS or MQTT dependency, achieving minimal control latency, deterministic 20 ms timing cycles, and seamless integration with Unitree's onboard motion controller (`Legged_sport`).

---

## 🚀 Key Features

- **Direct High-Level UDP SDK (Port 8082, 50 Hz)**: Bypasses heavy middleware stacks (ROS/ROS2/MQTT) by communicating directly with the robot's real-time motion controller at `192.168.123.161:8082`.
- **Closed-Loop Stair Climbing Heading Hold**:
  - Employs official crawl gait (`mode 2`, `gaitType 3`) with calibrated 8 cm step clearance (`footRaiseHeight = 0.08` delta).
  - Actively tracks and locks the robot's heading yaw via onboard IMU telemetry ($\Delta\psi = \text{atan2}(\sin(\psi_0 - \psi_{\text{live}}), \cos(\psi_0 - \psi_{\text{live}}))$).
  - Applies real-time proportional yaw rate corrections ($v_{yaw} = K_p \cdot \Delta\psi$, $K_p = 1.20$, clamped to $\pm 0.35$ rad/s) at 50 Hz, completely eliminating drift and lateral slip on stair climbs.
  - Right Stick X provides instant, seamless manual steering override.
- **Anti-Stumble Stance Completion**:
  - Implements continuous `Mode 2` velocity streaming with smooth slew-rate deceleration ($0.60\text{ m/s}^2$).
  - When the analog stick is released, the controller smoothly ramps velocity to zero without abruptly cutting to `Mode 1` (Force Stand), allowing the Convex MPC controller to complete the swing phase and plant all four feet squarely on stair treads.
- **Dynamic Continuous Body Height Adjustment**:
  - Hold **D-Pad UP** or **D-Pad DOWN** in `STAND` mode to continuously raise or lower the body height (range: $-0.10\text{ m}$ crouch to $+0.08\text{ m}$ tall stance) at a smooth $0.12\text{ m/s}$ ramp.
  - Automatically ramps height back to nominal ($0.0\text{ m}$) before transitioning into `WALK` or `STAIRS` to protect joint limits.
- **Proportional Pitch Tilt**:
  - Tilt the robot's body pitch up/down up to $\pm 0.35\text{ rad}$ ($\pm 20^\circ$) using **Right Stick Y** in `STAND` mode with spring-back auto-centering.
- **Official 4-Stage State Transitions**:
  - **Safe Sit-Down**: `Mode 1` (0.5 s) $\rightarrow$ `Mode 6` (1.0 s) $\rightarrow$ `Mode 5` (2.0 s) $\rightarrow$ `Mode 7` (1.0 s) $\rightarrow$ rests in `DAMP`.
  - **Safe Stand-Up**: `Mode 7` (0.5 s) $\rightarrow$ `Mode 5` (1.0 s) $\rightarrow$ `Mode 6` (2.0 s) $\rightarrow$ `Mode 1` (1.0 s) $\rightarrow$ locks in `STAND`.
- **BMS Battery Telemetry & Autonomous Safety**:
  - Reads real-time State of Charge (SOC) via custom `unitree_bms` pybind11 extension.
  - **Low Battery Protection ($\le 20\%$)**: Automatically initiates a controlled deceleration and smooth 4-stage sit-down into `DAMP` mode after a 1.0 s debounce.
  - **Low Battery Lockout ($\le 22\%$)**: Blocks standing up from the floor when battery charge is insufficient, preventing mid-ascent brownouts.
- **Failsafe & Reconnection Watchdog**:
  - If the Bluetooth connection drops mid-operation, the daemon immediately clamps commanded velocities to zero and holds a safe stance at 50 Hz until the controller reconnects.

---

## 🏛️ System Architecture

```
 +-------------------------------------------------------+
 |             PS4 DualShock 4 Controller                |
 |  (Analog Sticks, Face Buttons, L2/R2 Triggers, D-Pad) |
 +---------------------------+---------------------------+
                             | Bluetooth HID (evdev)
                             v
 +-------------------------------------------------------+
 |            Raspberry Pi 3B+ (192.168.12.1)            |
 |                                                       |
 |   /dev/input/js0 <--- Linux Joystick API              |
 |         |                                             |
 |         v                                             |
 |   unitree_go1_ps4_controller.py (50 Hz Daemon)        |
 |   - Chorded Mode FSM (DAMP, STAND, WALK, STAIRS)      |
 |   - Closed-Loop Stair Yaw Heading Hold (IMU feedback) |
 |   - Slew Rate Velocity Ramping & Deadzones            |
 |   - BMS Telemetry & Low-Battery Safety Guard          |
 |         |                                             |
 |         v                                             |
 |   robot_interface_high_level.so & unitree_bms.so      |
 +---------------------------+---------------------------+
                             | UDP Port 8082 (20 ms loop)
                             v
 +-------------------------------------------------------+
 |        Unitree Go1 Motion Controller (192.168.123.161)|
 |                 Binary: Legged_sport                  |
 |                                                       |
 |   - ConvexMPCLocomotion (Mode 2, gaitType 1 - Trot)   |
 |   - ConvexMPCLocomotionStairs (Mode 2, gaitType 3)    |
 |   - TouchDownEstimator & Leg Inverse Kinematics       |
 +-------------------------------------------------------+
```

---

## 🎮 Controller Button Mapping

### Mode Chords (Safety Dual-Button Combos)
To prevent accidental triggers, all major posture and locomotion mode switches require holding **`L2`** as a modifier:

| Button Combination | Target Mode | Description | Allowed From |
| :--- | :--- | :--- | :--- |
| **`L2` + `▲ (Triangle)`** | **`WALK`** | Dynamic trot gait (`gaitType 1`), 8 cm foot clearance, full 3-DOF velocity control | `STAND`, `STAIRS` |
| **`L2` + `✖ (Cross)`** | **`STAIRS`** | Stair climbing crawl (`gaitType 3`), 8 cm foot raise, closed-loop heading hold | `STAND`, `WALK` |
| **`L2` + `■ (Square)`** | **`STAND`** | Force stand posture lock (`mode 1`), Right Stick Y pitch tilt | `WALK`, `STAIRS` |
| **`L2` + `■ (Square)`** | **`STAND UP`** | Executes official 4-stage ascent (7 $\rightarrow$ 5 $\rightarrow$ 6 $\rightarrow$ 1) to standing | `DAMP`, `PRONE` |
| **`L2` + `⭕ (Circle)`** | **`SIT DOWN`** | Executes official 4-stage descent (1 $\rightarrow$ 6 $\rightarrow$ 5 $\rightarrow$ 7) to resting flat | `STAND` only |

### Continuous Adjustments (STAND Mode Only)

| Control | Function | Parameters |
| :--- | :--- | :--- |
| **D-Pad `UP` (Hold)** | Raise Body Height | Up to $+0.08\text{ m}$ at $0.12\text{ m/s}$ continuous ramp |
| **D-Pad `DOWN` (Hold)**| Lower Body Height | Down to $-0.10\text{ m}$ at $0.12\text{ m/s}$ continuous ramp |
| **Right Stick `Y`** | Pitch Tilt Up / Down | $\pm 0.35\text{ rad}$ ($\pm 20^\circ$) with spring auto-centering to $0.0\text{ rad}$ |

### Locomotion Analog Sticks

| Control | Locomotion Mode | Function | Range / Envelope |
| :--- | :--- | :--- | :--- |
| **Left Stick `Y`** | `WALK` | Forward / Backward Velocity ($V_x$) | $+1.50\text{ m/s}$ forward / $-1.00\text{ m/s}$ reverse |
| **Left Stick `X`** | `WALK` | Lateral Strafe Velocity ($V_y$) | $\pm 1.00\text{ m/s}$ left/right |
| **Right Stick `X`**| `WALK` | Turning Yaw Rate ($V_{yaw}$) | $\pm 2.50\text{ rad/s}$ turn rate |
| **Left Stick `Y`** | `STAIRS` | Ascent / Descent Velocity ($V_x$) | $+0.20\text{ m/s}$ ascent / $-0.15\text{ m/s}$ descent |
| **Left Stick `X`** | `STAIRS` | Lateral Strafe Velocity ($V_y$) | Locked to $0.00\text{ m/s}$ for crawl stability |
| **Right Stick `X`**| `STAIRS` | Manual Steering / Heading Lock | $\pm 0.50\text{ rad/s}$ (overrides closed-loop lock) |

---

## 📊 Live Visual Telemetry HUD

Every second, the controller outputs a formatted telemetry console display:

```text
  ╔════════════════════════════════════════════════════╗
  ║  MODE    : STAIRS    SDK mode=2   gait=3           ║
  ║  BATTERY :  82%  [████████░░]                      ║
  ╠════════════════════════════════════════════════════╣
  ║  CMD VEL : VX=+0.20  VY=+0.00  YAW=+0.04           ║
  ║  ACT VEL : VX=+0.19  VY=-0.01  VZ=+0.00 (m/s)      ║
  ╠════════════════════════════════════════════════════╣
  ║  PITCH   : cmd=+0.000 rad                          ║
  ║  IMU     : R=+0.012  P=-0.384  Y=+1.240 (rad)      ║
  ║  HEIGHT  : cmd=+0.000 m   actual=+0.285 m          ║
  ╠════════════════════════════════════════════════════╣
  ║  FEET    : FL=  85 FR=  92 RL= 110 RR=  78 N [XXXX]║
  ╚════════════════════════════════════════════════════╝
```

- **MODE & GAIT**: Current software state machine mode, firmware SDK mode, and active MPC gait type.
- **BATTERY**: State of charge with visual battery percentage indicator.
- **CMD vs ACT VEL**: Real-time velocity commanded vs estimated state from the onboard EKF.
- **IMU**: Live roll, pitch, and yaw angles from the robot's IMU.
- **FEET**: Ground reaction force (in Newtons) for Front-Left, Front-Right, Rear-Left, Rear-Right feet, with contact indicators (`X` = loaded $> 50\text{ N}$, `.` = swing/unloaded).

---

## 🛠️ Installation & Setup

### Prerequisites

- **Raspberry Pi 3B+ / 4** on the Unitree Go1 Air (IP: `192.168.12.1`).
- Python 3.7+ with standard libraries (`socket`, `struct`, `math`, `signal`, `logging`).
- Unitree High-Level SDK library: `robot_interface_high_level.cpython-37m-aarch64-linux-gnu.so`.
- Unitree BMS pybind11 module: `unitree_bms.cpython-37m-aarch64-linux-gnu.so`.

### 1. Bluetooth Pairing (DualShock 4)

Put the PS4 controller into pairing mode:
1. Press and hold the **Share** button and **PS Button** together until the light bar flashes rapid double-pulses.
2. On the Raspberry Pi, launch `bluetoothctl`:
   ```bash
   bluetoothctl
   [bluetooth]# agent on
   [bluetooth]# default-agent
   [bluetooth]# scan on
   # Locate Wireless Controller MAC address (e.g. 1C:A0:B8:xx:xx:xx)
   [bluetooth]# pair <MAC_ADDRESS>
   [bluetooth]# trust <MAC_ADDRESS>
   [bluetooth]# connect <MAC_ADDRESS>
   ```
3. Verify joystick device presence:
   ```bash
   ls -la /dev/input/js*
   ```

### 2. Deployment to Robot

Clone or sync the repository to the robot using the included sync script:

```bash
cd unitree_go1_ps4
./sync_to_pi.sh
```

The script automatically:
1. Connects to the robot's AP (`Unitree_Go421436A`).
2. Copies the Python controller, compiled `.so` modules, and systemd service file to `/home/pi/Unitree/ps4_controller/`.
3. Installs and restarts the systemd background daemon.

### 3. Systemd Service Setup

To manage the daemon manually on the Raspberry Pi:

```bash
# Enable service on boot
sudo systemctl enable /home/pi/Unitree/ps4_controller/unitree_go1_ps4.service

# Start or restart service
sudo systemctl restart unitree_go1_ps4.service

# Check service status
sudo systemctl status unitree_go1_ps4.service

# View live log output
journalctl -u unitree_go1_ps4.service -b -f -a --no-pager
```

---

## 🔬 Technical Insights: Reverse-Engineering `Legged_sport`

During the development of this teleoperation system, we disassembled and reverse-engineered the official Go1 motion controller binary (`Legged_sport`) to diagnose two critical locomotion issues:

### 1. Why STAIR Mode Drifts (and How Closed-Loop Heading Hold Fixes It)

- **Firmware Open-Loop Target Yaw Calculation**:
  Disassembly of `ConvexMPCLocomotionStairs::setupCommand` (`0x000ea500` - `0x000ea750`) revealed that the firmware computes target yaw at every 20 ms cycle directly from the instantaneous IMU reading:

  $$\text{target\_yaw} = \text{live\_yaw} + v_{\text{yaw}} \cdot \Delta t$$

- **The Drift Cause**:
  When ascending or descending stairs, physical step edge impacts, pitch inclination, and leg compliance introduce minor lateral forces. Because the firmware recalculates `target_yaw` from the current IMU angle on every tick rather than maintaining a fixed heading setpoint, the robot accumulates an open-loop drift of up to `-0.11 m/s`.

- **Closed-Loop Heading Hold Solution**:
  Our controller implements an active outer-loop heading lock at 50 Hz:
  - When moving forward or backward on stairs without manual steering, the initial heading setpoint $\psi_0$ is latched.
  - The controller calculates the shortest angular error wrapped in $[-\pi, \pi]$:

    $$\Delta\psi = \text{atan2}(\sin(\psi_0 - \psi_{\text{live}}), \cos(\psi_0 - \psi_{\text{live}}))$$

  - It injects a proportional corrective yaw rate ($v_{\text{yaw}} = K_p \cdot \Delta\psi$, with $K_p = 1.20$, clamped to $\pm 0.35\text{ rad/s}$), keeping the robot tracking in a straight line.
  - Manual steering input on Right Stick X takes instant priority and resets the target heading setpoint.

### 2. Why Mid-Motion Stumbling Occurred
- When stopping on stairs, cutting directly to `Mode 1` (Force Stand) aborted the Convex MPC crawl cycle mid-stride, freezing swinging legs in mid-air.
- **Solution**: The controller maintains `Mode 2, gaitType 3` continuously while decelerating to $V_x = 0, V_{yaw} = 0$. This allows the MPC controller's `TouchDownEstimator` to execute its complete four-phase gait cycle and ground all four feet firmly on the stair treads before settling.

### 3. Foot Raise Height Delta
- Header inspection of `comm.h` confirmed that `footRaiseHeight` is an additive delta above nominal ground clearance (nominal $= 0.08\text{ m}$).
- Setting `STAIR_FOOT_RAISE = 0.08` produces an optimal total step clearance of $\sim 16\text{ cm}$, easily clearing standard architectural stair risers without excessive vertical lift or roll instability.

---

## 📂 Repository Structure

```
├── unitree_go1_ps4_controller.py       # Main 50 Hz UDP teleoperation daemon
├── unitree_go1_ps4.service            # Systemd service configuration
├── sync_to_pi.sh                      # One-click deployment script
├── go1log                             # Shortcut live log viewing script
├── robot_interface_high_level.*.so    # Unitree High-Level UDP SDK library
├── unitree_bms.*.so                   # pybind11 BMS telemetry bindings
├── README.md                          # Documentation & controls reference
└── .gitignore                         # Python cache & build artifact exclusions
```

---

## 🛡️ License

This project is licensed under the Apache 2.0 License.
Developed for research and teleoperation of the Unitree Go1 Air quadruped robot.
