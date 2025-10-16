# hardware map for parallel plate bot
driver station config name: `parallel plate v0`
## control hub
### I2C Bus 0
| port |     device     | verbatim name |
|:-----|:---------------|:--------------|
| 0    | BN0055         | `imu`         |

make sure you set the device specifically to `REV internal IMU (BN0055)`, control hubs <2022 have older, less accurate and different ones that we don't want to use
### DC motors
| motor port | verbatim name |
|:-----------|:--------------|
| 1          | `frontLeft`   |
| 2          | `frontRight`  |
| 3          | `backLeft`    |
| 4          | `backRight`   |

