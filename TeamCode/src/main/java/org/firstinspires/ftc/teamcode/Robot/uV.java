package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
@Config
public class uV {

    public static double outtakePower = 1.0;
    public static double intakePower = 1;

    public static double outtakeServoClose = 0.27;
    public static double outtakeServoOpen = 0.05;

    public static double geckoLiftPower = 1;

    public static double camLeversDown = 0.25;

    public static double camLeversUp = 0.7;
}
