package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Config
@Configurable
public class uV {
    // Gate positions
    public static double gateOpen = 0.77;
    public static double gateClosed = 0.6;

    // Transfer
    public static double rollerOneP = 1;
    public static double rollerTwoP = 1;
    // lob positions
    public static double lobMax = 1;
    public static double lobMin = 0.25;

    // SHOOTER CONSTANTS
    public static double scoreHeight = 26;
    public static double scoreAngle = Math.toRadians(-30);
    public static double passThroughPointRadius = 5;


    public static double INTAKE_TIMEOUT_MS = 1000;
    public static double AUTOPARK_TIMEOUT_MS = 28_000;
    public static double STABILIZATION_TIMEOUT_MS = 400;
    public static double MINIMUM_SHOOT_TIMEOUT_MS = 2_000;

    public static boolean NN_LOGGING_ENABLE = false;
    public static boolean USE_NN_AIM_ASSIST = true;
}
