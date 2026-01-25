# St. Mark's FTC-A Decode Repo For Team 23381 "The Marksmen"

## ADB connect instructions

if `adb devices` doesn't show any devices:
1. close and reopen Android Studio
2. connect to driver hub using wifi
3. run `adb kill-server && adb start-server`
4. run `adb connect 192.168.43.1`
5. now try `adb devices`

it may take a second for the control hub to show up in Android Studio

## programming conventions

### units
- distance units:
  - hardware offsets: millimeters (easy to get from Onshape)
  - field distances: inches (used in Pedro Pathing)
- angle units:
  - limelight/vision: degrees
  - everything else: radians
- all speed units: TPS or radians/s
  - internal logic: TPS
  - human-facing: actual RPM of flywheel (accounting for ratio)

### odo distances
- robot length (back to front) is **420mm** / **16.53543"**
- robot width (left to right) is **400mm** / **15.74803"**

# hardware map

driver station config name: `v0 turret`

## control hub

### I²C ports/buses
| port/bus  | device type            | location                    | verbatim name         |
|:----------|:-----------------------|-----------------------------|:----------------------|
| `0`       | *unused*               | *unused*                    | *unused*              |
| `1`       | Pinpoint Odo Computer  | left side under control hub | `odo`                 |
| `2`       | REV 2M Distance Sensor | lower transfer              | `lowerTransferSensor` |
| `3`       | *unused*               | *unused*                    | *unused*              |

### digital ports
| port  | device type    | location       | verbatim name         |
|:------|:---------------|----------------|:----------------------|
| `0`   | *unused*       | *unused*       | *unused*              |
| `1`   | Digital Device | side of turret | `upperTransferSensor` |

the cable for upper transfer plugs into the `0/1` digital port, but the sensor must be configured as using `port 1`

### USB ports

| port    | device       | verbatim name |
|:--------|:-------------|:--------------|
| USB 3.0 | Limelight 3A | `limelight`   |

after scanning, the Limelight will show up as `Ethernet Device` under the USB devices, make sure to rename it to the verbatim name
make sure to save the config with the Limelight under a new name, as scanning may delete other devices

### DC motors

| motor port  | motor type              | verbatim name | encoder? |
|:------------|-------------------------|:--------------|:---------|
| `0`         | GoBILDA 5202/3/4 series | `frontLeft`   | ❌        |
| `1`         | GoBILDA 5202/3/4 series | `frontRight`  | ❌        |
| `2`         | GoBILDA 5202/3/4 series | `backLeft`    | ❌        |
| `3`         | GoBILDA 5202/3/4 series | `backRight`   | ❌        |

make sure to connect every motor with the correct polarity; the reversing should is done in software

### servos

| servo port   | servo type                | verbatim name   |
|:-------------|---------------------------|:----------------|
| `0`          | Servo                     | `lowerTransfer` |
| `1`          | Servo                     | `upperTransfer` |
| `2`          | Continuous Rotation Servo | `turret1`       |
| `3`          | Continuous Rotation Servo | `turret2`       |
| `4`          | Servo                     | `hood`          |
| `5`          | *unused*                  | *unused*        |

the connection order of turret1/2 doesn't matter, since they both face the same direction

## expansion hub

### connection method (ports matter)
![expansion hub connection](doc/media/expansionHubConnection.png)

### I2C ports/buses

| port/bus | device type            | location       | verbatim name         |
|:---------|:-----------------------|----------------|:----------------------|
| 0        | REV 2M Distance Sensor | side of intake | `intakeSensor`        |
| 1        | *unused*               | *unused*       | *unused*              |
| 2        | *unused*               | *unused*       | *unused*              |
| 3        | *unused*               | *unused*       | *unused*              |

### DC motors

| motor port   | motor type              | verbatim name   | encoder?  |
|:-------------|-------------------------|:----------------|:----------|
| `0`          | GoBILDA 5202/3/4 series | `intake`        | ❌         |
| `1`          | GoBILDA 5202/3/4 series | `launchLeft`    | ✅         |
| `2`          | GoBILDA 5202/3/4 series | `launchRight`   | ✅         |
| `3`          | GoBILDA 5202/3/4 series | `turretEncoder` | ✅         |

`turretEncoder` shouldn't have a motor connected, because we are just using the encoder port
connecting `launchLeft`/`launchRight` correctly is very important
- you only need an encoder on one of them to function, but having both increases accuracy+reliability

# controller map

## sticks

### left stick

- **X**: move robot left/right
- **Y**: move robot forwards/back

the **start** button toggles between robot/field centric control

### right stick:

- **X**: rotate clockwise/counterclockwise
- **Y**: *unused*

## bumpers

- **left bumper**: lock turret
- **right bumper**: launch 1ball

## triggers

- **left trigger**: slow mode 
- **right trigger**: *unused*

## buttons

### face buttons
- **A**: toggle intake
- **B**: toggle manual/automatic launch
  - **auto**: uses vision/odo to find out needed launch speed to reach goal
  - **manual**: sets launch speed to constant defined in Tunables.java (calibrated for shooting from crease)
    - the setpoint can be changed with the d-pad buttons
- **Y**: launch 3 balls
- **X**: reverse intake

### d-pad
- **up**: increase launch RPM setpoint
- **down**: decrease launch RPM setpoint
- **left**: decrease launch RPM setpoint by half
- **right**: increase launch RPM setpoint by half

### other buttons
- **start**: toggle field/robot centric
- **back**: reset field centric heading
  - make sure to orient the robot towards the top of the field (in between the goals)

# OpModes

## TeleOp
- `Enable/Disable Panels`: enables/disables [Panels telemetry](http://192.168.43.1:8001/)
- `BlueTeleOp`: TeleOp for blue team
- `RedTeleOp`: TeleOp for red team

## Auto

### blue team

- `BlueTriAuto`: starting by bottom triangle
- `BlueGoalAuto`: starting by blue team goal

### red team

- `RedTriAuto`: starting by bottom triangle
- `RedGoalAuto`: starting by red team goal

## Util

- `FlywheelTuner`: tune flywheel PIDF
- `LaunchTuner`: tune launch delays
- `SensorTuner`: test/tune distance sensors and their trigger points
- `ServoTuner`: test/tune servo endpoints
- `TurnTest`: test Pedro Pathing turning behaviour
- `TurretTuner`: test/tune turret encoder and PIDF
- `VisionTest`: test/tune limelight

# state machines

## auto states

1. `START`
2. `TRAVEL_TO_LAUNCH`
3. `LAUNCH`
4. `TRAVEL_TO_BALLS`
5. `RELOAD`
6. `GO_TO_CLEAR`
7. `CLEAR`
8. `GO_TO_END`
9. `END`

### ball triplets scored
- starts at 0
- increments every launch

## launch states

1. `OPENING_UPPER_TRANSFER`
2. `PUSHING_LOWER_TRANSFER`
3. `WAITING_FOR_EXIT`

### balls remaining

- starts at 3 (all in robot)
- decrements by 1 every ball that is launched

# vision

We are using a Limelight 3A

## pipelines

| filename         | index  | allowed tag IDs | purpose                                 |
|:-----------------|--------|:----------------|:----------------------------------------|
| `0_Obelisk.vpr`  | `0`    | 21, 22, 23      | detect pattern                          |
| `1_BlueTeam.vpr` | `1`    | 20              | get blue goal april tag distance and tx |
| `2_RedTeam.vpr`  | `2`    | 24              | get red goal april tag distance and tx  |

pipeline files are saved in the [limelight folder](limelight/)