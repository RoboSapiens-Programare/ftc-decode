package org.firstinspires.ftc.teamcode.Auto.Pathing; /* ============================================================= *
                                                      *           Pedro Pathing Visualizer — Auto-Generated           *
                                                      *                                                               *
                                                      *  Version: 1.6.2.                                              *
                                                      *  Copyright (c) 2026 Matthew Allen                             *
                                                      *                                                               *
                                                      *  THIS FILE IS AUTO-GENERATED — DO NOT EDIT MANUALLY.          *
                                                      *  Changes will be overwritten when regenerated.                *
                                                      * ============================================================= */

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BluePathing {

    public PathChain goToGrab0;
    public PathChain grab0;
    public PathChain shoot0;
    public PathChain goToGrab1;
    public PathChain grab1;
    public PathChain shoot1;
    public PathChain goToGrab2;
    public PathChain grab2;
    public PathChain shoot2;
    public PathChain leave;

    public BluePathing(Follower follower) {
        goToGrab0 =
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(62.880, 9.251),
                                        new Pose(43.755, 28.422),
                                        new Pose(35.000, 35.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                        .build();

        grab0 =
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(35.000, 35.000),
                                        new Pose(25.786, 37.057),
                                        new Pose(16.000, 35.000)))
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();

        shoot0 =
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(16.000, 35.000),
                                        new Pose(39.740, 33.740),
                                        new Pose(50.748, 25.649),
                                        new Pose(59.000, 20.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(116))
                        .build();

        goToGrab1 =
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(59.000, 20.000),
                                        new Pose(47.020, 44.525),
                                        new Pose(53.922, 30.041),
                                        new Pose(47.193, 60.030),
                                        new Pose(35.000, 60.000)))
                        .setTangentHeadingInterpolation()
                        .build();

        grab1 =
                follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(35.000, 60.000), new Pose(16.000, 60.000)))
                        .setTangentHeadingInterpolation()
                        .build();

        shoot1 =
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(16.000, 60.000),
                                        new Pose(64.936, 63.386),
                                        new Pose(45.000, 95.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                        .build();

        goToGrab2 =
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(45.000, 95.000),
                                        new Pose(52.701, 79.302),
                                        new Pose(35.000, 85.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                        .build();

        grab2 =
                follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(35.000, 85.000), new Pose(17.000, 85.000)))
                        .setTangentHeadingInterpolation()
                        .build();

        shoot2 =
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(17.000, 85.000),
                                        new Pose(30.693, 91.131),
                                        new Pose(32.222, 90.466),
                                        new Pose(24.938, 89.618),
                                        new Pose(45.000, 95.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                        .build();

        leave =
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(45.000, 95.000),
                                        new Pose(34.981, 96.863),
                                        new Pose(20.000, 95.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270))
                        .build();
    }
}
