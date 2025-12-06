package org.firstinspires.ftc.teamcode.Auto.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Odo test", group = "1. Auto Tests")
public class OdoTest extends OpMode {
    private DcMotorEx testMotor;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void init() {

        testMotor = hardwareMap.get(DcMotorEx.class, "testMotor");
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        dashboardTelemetry.addData("pos", testMotor.getCurrentPosition());
        dashboardTelemetry.update();
    }
}
