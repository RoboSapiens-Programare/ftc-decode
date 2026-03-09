package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class uV {
    // Intake pivot positions
    public static double intakeMid = 0.485;
    public static double intakeDown = 0.44;

    // Gate positions
    public static double gateOpen = 0.52;
    public static double gateClosed = 0.35;

    // Transfer
    public static double rollerOneP = 1;
    public static double rollerTwoP = 1;

    // Outtake

    // PIDF values for odometry tracking
    public static double odometryKp = 0.01;
    public static double odometryKi = 0;
    public static double odometryKd = 0.000012;
    public static double odometryKf = 0.01;
    public static double odometryKpF = 0.01;
    public static double odometryKiF = 0;
    public static double odometryKdF = 0;
    public static double odometryKfF = 0.01;

    // PIDF values for limelight tracking
    public static double limelightKp = 0.005;
    public static double limelightKi = 0.0003;
    public static double limelightKd = 0.00000002;
    public static double limelightKf = 0.04;
    public static double limelightKpF = 0.0035;
    public static double limelightKiF = 0.0003;
    public static double limelightKdF = 0.00000002;
    public static double limelightKfF = 0.02;

    // lob positions
    public static double lobMax = 1;
    public static double lobMin = 0.25;

    public static double tuningVel = 1400;
    public static double tuningLob = 1;


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
