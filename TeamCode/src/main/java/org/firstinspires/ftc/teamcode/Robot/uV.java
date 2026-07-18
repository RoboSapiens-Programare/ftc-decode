package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Config
@Configurable
public class uV {
    // Gate positions
    public static double gateOpen = 0.425;
    public static double gateMid = 0.55;
    public static double gateClosed = 0.408;

    // Transfer
    public static double rollerOneP = 1;
    public static double rollerTwoP = 1;

    public static double INTAKE_TIMEOUT_MS = 1800;
    public static double AUTOPARK_TIMEOUT_MS = 29_000;
    public static double STABILIZATION_TIMEOUT_MS = 100;
    public static double GATE_PICKUP_WAIT = 3000;
    public static double MINIMUM_SHOOT_TIMEOUT_MS = 500;

    public static boolean NN_LOGGING_ENABLE = false;
    public static boolean USE_NN_AIM_ASSIST = true;

    public static boolean USE_VELOCITY_REGRESSION = true;
}
