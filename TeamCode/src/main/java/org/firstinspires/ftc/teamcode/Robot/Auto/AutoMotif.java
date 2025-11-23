/*
Hello :D my names mikey and i'm the head of software on team 21721. I was looking at the april tag sample code on the PP (pedro pathing) website and it kinda confused me or just wasn't
what I needed to do, so I decided to make my own! Before you worry about the code itself u need to know a bit about April tags. April tags are basically just QR codes; in the sense
that when you scan them they give u a numerical value. the april tag values for this season are as the following-

Blue Goal: 20
Motif GPP: 21
Motif PGP: 22
Motif PPG: 23
Red Goal: 24

So basically, you lineup your robot in front of the motif april tag. It scans said April Tag and then gives you a value back. You then have three if/then statements where you pretty much
say "if the numeric value is 21, then run the GPP pathbuilder" and so on. Right now, though, the code just has movement. So whenever you get your shooting and intake mechanisms figured out, just add that code in the
designated function and call the function in whichever part of the pathbuilder it is needed. I hope this helps!
*/

package org.firstinspires.ftc.teamcode.Robot.Auto;

// FTC SDK

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "April Tag with PP test", group = "Autonomous")
@Config
@SuppressWarnings(
        "FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class AutoMotif extends LinearOpMode {
    private Robot robot = new Robot(hardwareMap);
    // Initialize elapsed timer
    private final ElapsedTime runtime = new ElapsedTime();

    // Initialize poses
    private final Pose startPose =
            new Pose(72, 120, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose =
            new Pose(
                    72,
                    20,
                    Math.toRadians(
                            115)); // Scoring Pose of our robot. It is facing the goal at a 115
    // degree angle.
    private final Pose PPGPose =
            new Pose(
                    100,
                    83.5,
                    Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose PGPPose =
            new Pose(
                    100,
                    59.5,
                    Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose GPPPose =
            new Pose(
                    100,
                    35.5,
                    Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    // Initialize variables for paths

    private PathChain grabPPG;
    private PathChain scorePPG;
    private PathChain grabPGP;
    private PathChain scorePGP;
    private PathChain grabGPP;
    private PathChain scoreGPP;
    private PathChain scorePreload;

    private boolean singleton1 = true;
    private boolean singleton2 = true;
    private boolean singleton3 = true;
    private boolean singleton4 = true;

    private final ElapsedTime Timer = new ElapsedTime();

    // set April Tag values to specific patterns
    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;
    private static final boolean USE_WEBCAM =
            true; // Set true to use a webcam, or false for a phone camera
    private VisionPortal visionPortal; // Used to manage the video source.
    private AprilTagProcessor aprilTag; // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null; // Used to hold the data for a detected AprilTag

    private org.firstinspires.ftc.teamcode.TeleOP.FSM FSM =
            new org.firstinspires.ftc.teamcode.TeleOP.FSM();

    // Other variables
    private Pose currentPose;
    private Follower follower; // Pedro Pathing follower
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();

    private int pathState;
    private int pathStatePPG; // Current state machine value
    private int pathStatePGP; // Current state machine value
    private int pathStateGPP; // Current state machine value

    private int foundID; // Current state machine value, dictates which one to run

    // Custom logging function to support telemetry and Panels
    private void log(String caption, Object... text) {
        if (text.length == 1) {
            telemetry.addData(caption, text[0]);
        } else if (text.length >= 2) {
            StringBuilder message = new StringBuilder();
            for (int i = 0; i < text.length; i++) {
                message.append(text[i]);
                if (i < text.length - 1) message.append(" ");
            }
            telemetry.addData(caption, message.toString());
        }
    }

    // a place to put your intake and shooting functions
    public void intakeArtifacts() {
        robot.intake.startMotor();
        robot.intake.update();
        // Put your intake logic/functions here
    }

    public void shootArtifacts() {
        // Put your shooting logic/functions her
    }

    @Override
    public void runOpMode() {
        telemetry.update();
        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        boolean targetFound = false; // Set to true when an AprilTag target is detected
        initAprilTag();

        if (USE_WEBCAM) {
            setManualExposure(6, 250); // Use low exposure time to reduce motion blur
        }

        // Log completed initialization to Panels and driver station (custom log function)
        log("Status", "Initialized");
        telemetry.update(); // Update driver station after logging

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        setpathState(0);
        runtime.reset();

        while (opModeIsActive()) {
            // Update Pedro Pathing and Panels every iteration
            follower.update();
            telemetry.update();
            currentPose = follower.getPose(); // Update the current pose
            targetFound = false;
            desiredTag = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    buildPathsAny();
                    //  Check to see if we want to track towards this tag.
                    if (detection.id == PPG_TAG_ID) {
                        targetFound = true;
                        desiredTag = detection;
                        foundID = 21; // This should likely be PPG_TAG_ID or the corresponding state
                        // machine ID
                        break; // don't look any further.
                    } else if (detection.id == PGP_TAG_ID) {
                        targetFound = true;
                        desiredTag = detection;
                        foundID = 22; // This should likely be PGP_TAG_ID or the corresponding state
                        // machine ID
                        break; // don't look any further.
                    } else if (detection.id == GPP_TAG_ID) {
                        targetFound = true;
                        desiredTag = detection;
                        foundID = 23; // This should likely be GPP_TAG_ID or the corresponding state
                        // machine ID
                        break; // don't look any further.
                    }
                } else {
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            updateStateMachineAny();

            // sorting in here ??

            if (foundID == 21) {

            } else if (foundID == 22) {

            } else if (foundID == 23) {

            }

            // Log to Panels and driver station (custom log function)
            log("Elapsed", runtime.toString());
            log("X", currentPose.getX());
            log("Y", currentPose.getY());
            log("Heading", currentPose.getHeading());
            telemetry.update(); // Update the driver station after logging
        }
    }

    public void buildPathsAny() {

        // ideally startPose should be scorePose to save time
        // here you make the first path just in case you change the startPose to a pos where you
        // cant
        // score

        scorePreload =
                follower.pathBuilder()
                        .addPath(new BezierLine(startPose, scorePose))
                        .setLinearHeadingInterpolation(
                                startPose.getHeading(), scorePose.getHeading())
                        .build();

        grabPPG =
                follower.pathBuilder()
                        .addPath(new BezierLine(scorePose, PPGPose))
                        .setLinearHeadingInterpolation(scorePose.getHeading(), PPGPose.getHeading())
                        .build();

        scorePPG =
                follower.pathBuilder()
                        .addPath(new BezierLine(PPGPose, scorePose))
                        .setLinearHeadingInterpolation(PPGPose.getHeading(), scorePose.getHeading())
                        .build();

        grabPGP =
                follower.pathBuilder()
                        .addPath(new BezierLine(scorePose, PGPPose))
                        .setLinearHeadingInterpolation(scorePose.getHeading(), PGPPose.getHeading())
                        .build();

        scorePGP =
                follower.pathBuilder()
                        .addPath(new BezierLine(PGPPose, scorePose))
                        .setLinearHeadingInterpolation(PGPPose.getHeading(), scorePose.getHeading())
                        .build();

        grabGPP =
                follower.pathBuilder()
                        .addPath(new BezierLine(scorePose, GPPPose))
                        .setLinearHeadingInterpolation(scorePose.getHeading(), GPPPose.getHeading())
                        .build();

        scoreGPP =
                follower.pathBuilder()
                        .addPath(new BezierLine(GPPPose, scorePose))
                        .setLinearHeadingInterpolation(GPPPose.getHeading(), scorePose.getHeading())
                        .build();
    }

    public void updateStateMachineAny() {
        switch (pathStatePPG) {
            case 0:
                follower.followPath(scorePreload);
                robot.turret.update();

                if (!follower.isBusy()) {}

                setpathState(1);
                break;

                //            case 1:
                //
                //                if (!follower.isBusy()) {
                //                    follower.followPath(grabPPG);
                //                    setpathState(2);
                //                }
                //                break;
                //
                //            case 2:
                //
                //
                //
                //                if (!follower.isBusy()) {
                //                    follower.followPath(scorePPG);
                //                    setpathState(3);
                //                }
                //                break;
                //
                //            case 3:
                //                if (!follower.isBusy()) {
                //                    follower.followPath(grabPGP);
                //                    setpathState(4);
                //                }
                //                break;
                //
                //            case 4:
                //                if (!follower.isBusy()) {
                //                    follower.followPath(scorePGP);
                //                    setpathState(5);
                //                }
                //                break;
                //
                //            case 5:
                //                if (!follower.isBusy()) {
                //                    follower.followPath(grabGPP);
                //                    setpathState(6);
                //                }
                //                break;
                //
                //            case 6:
                //                if (!follower.isBusy()) {
                //                    follower.followPath(scoreGPP);
                //                    setpathState(7);
                //                }
                //                break;
                //
                //            case 7:
                //                // finished
                //                // TODO: add "parking" if you have time
                //                break;
        }
    }

    // Setter methods for pathState variables placed at the class level
    void setpathState(int newPathState) {
        this.pathState = newPathState;
    }

    /** start the AprilTag processor. */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal =
                    new VisionPortal.Builder()
                            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                            .addProcessor(aprilTag)
                            .build();
        } else {
            visionPortal =
                    new VisionPortal.Builder()
                            .setCamera(BuiltinCameraDirection.BACK)
                            .addProcessor(aprilTag)
                            .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested()
                    && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        // ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        // if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
        //     exposureControl.setMode(ExposureControl.Mode.Manual);
        //     sleep(50);
        // }
        // exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        // sleep(20);
        // GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        // gainControl.setGain(gain);
        // sleep(20);
    }
}
