package org.firstinspires.ftc.teamcode;

public class Launcher {
    private enum LaunchState { // shooting ball states (we want this to be non-blocking)
        START,
        OPEN_UPPER_TRANSFER,
        PUSH_LOWER_TRANSFER,
        WAIT_FOR_PUSH,
        END
    }
}
