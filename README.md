# St. Mark's FTC-A Decode Repo
## ADB connect instructions
if `adb devices` doesn't show any devices:
1. connect to driver hub using wifi
2. use `adb connect 192.168.43.1`
3. now try `adb devices`

## programming conventions
### units
- all distance units: millimeters
- all angle units: radians
- all speed units: TPS or radians/s
  - TPS for commanding motors directly
  - radians/s for math (bcuz it's easy for physics)
### odo distances
- robot length (back to front) is **420mm**
- robot width (left to right) is **400mm**
# Hardware Map

driver station config name: `parallel plate v0`

## control hub

### I2C Bus 0
| port | device                | verbatim name |
|:-----|:----------------------|:--------------|
| 0    | BN0055                | `imu`         |
| 1    | Pinpoint Odo Computer | `odo`         |

(the `BN0055` is internal so you don't need to *physically* connect it)
make sure you set the device specifically to `REV internal IMU (BN0055)`, control hubs <2022 have older, less accurate and different ones that we don't want to use

### USB port

| port | device        | verbatim name |
|:-----|:--------------|:--------------|
| USB  | Logitech C290 | `camera`      |

### DC motors

| motor port | verbatim name | encoder? |
|:-----------|:--------------|:---------|
| 0          | `frontLeft`   | ❌        |
| 1          | `frontRight`  | ❌        |
| 2          | `backLeft`    | ❌        |
| 3          | `backRight`   | ❌        |

### servos

| servo port  | verbatim name   |
|:------------|:----------------|
| 0           | `lowerTransfer` |
| 1           | `upperTransfer` |
| 2           | none            |
| 3           | none            |
| 4           | none            |
| 5           | none            |


## expansion hub

### DC motors

| motor port | verbatim name | encoder? |
|:-----------|:--------------|:---------|
| 0          | `intake`      | ❌        |
| 1          | `launch`      | ✅        |
| 2          | none          | ❌        |
| 3          | none          | ❌        |

# Controller Map
## Sticks
### left stick
- X: move X (absolute coordinates)
- Y: move Y (absolute coordinates)

### right stick:
- X: rotate clockwise/counterclockwise
- Y: not used

## bumpers
- left bumper: slow mode
- right bumper: launch ball

## triggers
- left trigger: not used
- right trigger: launch power (when manually controlled)

## buttons
- A: toggle intake
- B: toggle launch (speed controlled with RY)
- Y: switch between manual and automatic control of launch velocity
- X: turn to goal or cancel turning to goal

- start: toggle field/robot centric

# multipliers
when moving around the field and back to the same place, our position was within ~0.25" so we prob won't need to recalibrate the pinpoint
## forward (48") tests
1. 19839.220040170083
2. 24006.84554319875
3. 24431.005099513706
4. 24949.657374839462
5. 25468.686568540168
- avg: _____

## lateral (48") tests
### going left
1. -24898.2422102001
2. -25300.129015353705
### going right
1. 26475.23513389474
2. 25923.59967956919

## turning
1. -1.0046447450070208
2. 1.0092479734478739

## forward velocity
1. 57.62592321681226
2. 57.19646592027559

## strafe velocity
1. 47.61915456216167
2. 46.422831377645174

## forward zero power acceleration (deceleration)
1. -46.364970304550404
2. -43.0153804561628

## lateral zero power acceleration (deceleration)
1. -38.25290065185062
2. -33.778581054706926

# OpModes
## TeleOp
- `BlueTeleOp`: blue team goal position for launch calculations
- `RedTeleOp`: red team goal position for launch calculations

## Auto
### Blue Team
- `BlueTriAuto`: starting by bottom triangle
- `BlueGoalAuto`: starting by blue team goal
### Red Team
- `RedTriAuto`: starting by bottom triangle
- `RedGoalAuto`: starting by red team goal

## other/util
- `FlywheelServoTest`: test servo endpoints and max flywheel speed

# State Machine
## main states
1. START
2. TRAVEL_TO_LAUNCH
3. LAUNCH
4. TRAVEL_TO_BALLS
5. RELOAD
6. GO_TO_END
7. END

## ball triplets remaining
- starts at 4 (1 in robot, 3 on field)
- decrements every launch