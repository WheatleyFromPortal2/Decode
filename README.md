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

### I²C ports/buses
| port/bus | device                 | location                     | verbatim name          |
|:---------|:-----------------------|------------------------------|:-----------------------|
| 1        | Pinpoint Odo Computer  | left side under control hub  | `odo`                  |

having sensors 1/2 on the left/right doesn't matter because we are just comparing values

### USB port

| port    | device       | verbatim name |
|:--------|:-------------|:--------------|
| USB 3.0 | Limelight 3A | `limelight`   |

the Limelight will show up as `Ethernet Device` under the USB devices, make sure to rename it to the verbatim name

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

### connection method (ports matter)
![expansion hub connection](doc/media/expansionHubConnection.png)

### I2C ports/buses
| port/bus | device                 | location                    | verbatim name         |
|:---------|:-----------------------|-----------------------------|:----------------------|
| 0        | REV 2M Distance Sensor | intake                      | `intakeSensor`        |
| 1        | REV Color Sensor V3    | lower transfer              | `lowerTransferSensor` |
| 2        | REV 2M Distance Sensor | left side of lower transfer | `upperTransferSensor` |
| 3        | *unused*               | *unused*                    | *unused*              |

having sensors 1/2 on the left/right doesn't matter because we are just comparing values

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
- left bumper: auto turn to goal
- right bumper: launch ball

## triggers
- left trigger: slow mode
- right trigger: launch power (when manually controlled)

## buttons
### face buttons
- A: toggle intake
- B: toggle manual/automatic launch
  - **auto**: uses odo to find out needed launch speed to reach goal
  - **manual**: sets launch speed to constant defined in Tunables.java (calibrated for shooting from crease)
- Y: launch 3 balls
- X: reverse intake

### d-pad
- up: increase launch RPM setpoint
- down: decrease launch RPM setpoint

### other buttons
- start: toggle field/robot centric
- back: reset field centric heading

# OpModes
## TeleOp
- `BlueTeleOp`: TeleOp for blue team
- `RedTeleOp`: TeleOp for red team

## Auto
### Blue Team
- `BlueTriAuto`: starting by bottom triangle
- `BlueGoalAuto`: starting by blue team goal
### Red Team
- `RedTriAuto`: starting by bottom triangle
- `RedGoalAuto`: starting by red team goal

## util
- `FlywheelTuner`: tune flywheel PIDF
- `LaunchTuner`: tune launch delays
- `SensorTest`: test distance sensors and their trigger points
- `ServoTest`: test servo endpoints
- `TurnTest`: test Pedro Pathing turning behaviour

# State Machines
## auto states
1. START
2. TRAVEL_TO_LAUNCH
3. LAUNCH
4. TRAVEL_TO_BALLS
5. RELOAD
6. GO_TO_END
7. END

### ball triplets remaining
- starts at 4 (1 in robot, 3 on field)
- decrements every launch

## launch states
1. OPENING_UPPER_TRANSFER
2. PUSHING_LOWER_TRANSFER
3. WAITING_FOR_EXIT

### balls remaining
- starts at 3 (all in robot)
- decrements by 1 every ball that is launched
- reset to 3 and return true when it hits 0

# vision

We are using a Limelight 3A

## Pipelines
| pipeline | allowed tag IDs | purpose                                      |
|:---------|:----------------|:---------------------------------------------|
| 0        | 21, 22, 23      | detect pattern                               |
| 1        | 20              | get blue goal launch distance and turn to it |
| 2        | 24              | get red goal launch distance and turn to it  |