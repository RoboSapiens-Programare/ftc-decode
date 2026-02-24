package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class uV {

    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;

    // Intake Servo Values
    public static double intakeLeftMid = 0.52;
    public static double intakeRightMid = 0.42;

    public static double intakeDownLeft = 0.55;
    public static double intakeDownRight = 0.45;

    public static double gateOpen = 0.9;
    public static double gateClosed = 0.8;

    // Transfer
    public static double rollerOneP = 1;
    public static double rollerTwoP = 1;

    // Outtake

    public static double odometryKp = 0;
    public static double odometryKi = 0;
    public static double odometryKd = 0;
    public static double odometryKf = 0;

    public static double limelightKp = 0;
    public static double limelightKi = 0;
    public static double limelightKd = 0;
    public static double limelightKf = 0;
}
