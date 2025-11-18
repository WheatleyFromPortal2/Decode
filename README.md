# St. Mark's FTC-A Decode Repo For Team 23381 "The Marksmen"
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
- robot length (back to front) is **420mm** / **16.53543"**
- robot width (left to right) is **400mm** / **15.74803"**
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

make sure to connect every motor with the correct polarity, the reversing should be done in software

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
- left bumper: not used
- right bumper: launch ball

## triggers
- left trigger: slow mode
- right trigger: launch power (when manually controlled)

## buttons
### face buttons
- A: toggle intake
- B: unused
- Y: switch between manual and automatic control of launch velocity
- X: turn to goal or cancel turning to goal
 
### d-pad
- up: auto orient for launch

### other buttons
- start: toggle field/robot centric
- back: reset field centric heading


# OpModes
## TeleOp
- `BozoTeleOp`: TeleOp for everything, current pose and goal pose should be automatically transferred

## Auto
### Blue Team
- `BlueTriAuto`: starting by bottom triangle
- `BlueGoalAuto`: starting by blue team goal
### Red Team
- `RedTriAuto`: starting by bottom triangle
- `RedGoalAuto`: starting by red team goal

## util
- `FlywheelTest`: test flywheel max speed
- `ServoTest`: test servo endpoints

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