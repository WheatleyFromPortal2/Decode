# St. Mark's FTC-A Decode Repo
## ADB connect instructions
if `adb devices` doesn't show any devices:
1. connect to driver hub using wifi
2. use `adb connect 192.168.43.1`
3. now try `adb devices`

# Hardware Map

driver station config name: `parallel plate v0`

## control hub

### I2C Bus 0
| port |     device     | verbatim name |
|:-----|:---------------|:--------------|
| 0    | BN0055         | `imu`         |

make sure you set the device specifically to `REV internal IMU (BN0055)`, control hubs <2022 have older, less accurate and different ones that we don't want to use

### USB port

| port | device        | verbatim name |
|:-----|:--------------|:--------------|
| USB  | Logitech C290 | `camera`      |

### DC motors

| motor port | verbatim name |
|:-----------|:--------------|
| 0          | `frontLeft`   |
| 1          | `frontRight`  |
| 2          | `backLeft`    |
| 3          | `backRight`   |

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

| motor port | verbatim name |
|:-----------|:--------------|
| 0          | `intake`      |
| 1          | `launch`      |
| 2          | none          |
| 3          | none          |

# Controller Map
## Sticks
left stick:
- X: move X (absolute coordinates)
- Y: move Y (absolute coordinates)
right stick:
- X: rotate clockwise/counterclockwise
- Y: launch power (experimental)

## bumpers
