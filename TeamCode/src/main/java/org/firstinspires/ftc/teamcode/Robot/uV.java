package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class uV {
    // Intake pivot positions
    public static double intakeMid = 0.485;
    public static double intakeDown = 0.44;

    // Gate positions
    public static double gateOpen = 0.8;
    public static double gateClosed = 0.7;

    // Transfer
    public static double rollerOneP = 1;
    public static double rollerTwoP = 1;

    // Outtake

    // PIDF values for odometry tracking
    public static double odometryKp = 0.00023;
    public static double odometryKi = 0.000321;
    public static double odometryKd = 0.000027;
    public static double odometryKf = 0.06;

    // PIDF values for limelight tracking
    public static double limelightKp = 0.004852;
    public static double limelightKi = 0.0067829;
    public static double limelightKd = 0.004592;
    public static double limelightKf = 0.0658;

    // lob positions
    public static double lobMax = 1;
    public static double lobMin = 0.25;


    // SHOOT ASSIST
    public static double llDistanceOne = 0.5;
    public static double llDistanceTwo = 1.1;

    public static double odometruDistanceOne = 0;
    public static double odometruDistanceTwo = 0;

    public static int velocityOne = 1300;
    public static int velocityTwo = 1600;

    public static double angleOne = 0.45;
    public static double angleTwo = 0.2;


    public static double llDistanceFar = 1.58;
    public static double odometruDistanceFar = 0;
    public static int velocityFar = 1870;
    public static double angleFar = 0.2;

}
