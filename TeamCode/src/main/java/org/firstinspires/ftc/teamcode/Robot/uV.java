package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class uV {

    public static double drivePower = 1;

    public static double outtakePower = 0.8;
    public static double intakePower = 1;

    public static double liftUp = 0.2;
    public static double liftDown = 0.6;

    public static double camLeversDown2 = 0.17;
    public static double camLeversDown1 = 0.23;

    public static double camLeversUp2 = 0.39;
    public static double camLeversUp1 = 0.45;

    public static double revolverPower = 0.08;

    public static int ticksPerRevolution = 8192;
    public static int revolverPositonIntake0 = 0;
    public static int revolverPositonOuttake0 = ticksPerRevolution / 2;


    public static double shootVelocity = 0;
}
