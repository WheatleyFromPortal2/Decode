package org.firstinspires.ftc.teamcode.subsys;

public class Fusion {
    /*
    - take in data from Pedro Pathing estimated position and Limelight position
    - use Kalman filter to intelligently combine
    - return result

    - we shouldn't need to update Pedro Pathing's position, since our TeleOp will just rely upon this and auto uses odo only
    - need to test whether this still works during a failure with the limelight
        - may need to add code to detect bad data from limelight and reject it
        - maybe have a maximum allowed error from vision?
     */
}
