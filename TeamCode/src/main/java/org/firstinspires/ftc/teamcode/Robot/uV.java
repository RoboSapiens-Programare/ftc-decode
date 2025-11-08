package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class uV {

    public static double outtakePower = 0.8;
    public static double intakePower = 1;

    public static double liftUp = 0.2;
    public static double uppiesDown = 0.6;

    public static double camLeversDown2 = 0.17;
    public static double camLeversDown1 = 0.23;

    public static double camLeversUp2 = 0.39;
    public static double camLeversUp1 = 0.45;

    public static double revolverPower = 0.08;

    public static double ticksPerRevolution = 8192; // TODO: CHECK
    public static double revolverPositonIntake0 = 0;
    public static double revolverPositonOuttake0 = ticksPerRevolution / 2;


    public static double revolverKp = 0.0;
    public static double revolverKi = 0.0;
    public static double revolverKd = 0.0;


}
