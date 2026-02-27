package org.firstinspires.ftc.teamcode.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import java.util.List;

@TeleOp(name = "TeleOpTest (World Class)", group = "Decode")
public class TeleOpTest extends OpMode {

    private Robot robot;

    // ==========================================
    // 1. OPTIMIZĂRI HARDWARE (Latență 0)
    // ==========================================
    private List<LynxModule> allHubs;
    private VoltageSensor batteryVoltageSensor;
    private final double NOMINAL_VOLTAGE = 13.0; // Voltajul la care robotul e perfect calibrat
    private double currentVoltage = 13.0;
    private double vComp = 1.0;

    // ==========================================
    // 2. FSM (MAȘINA DE STĂRI)
    // ==========================================
    public enum State {
        SEARCH_AND_DESTROY, // Colectare
        TRANSIT,            // Avem 3 bile, fugim spre zonă
        ENGAGING,           // Suntem în zonă, facem lock-on și turăm shooterul
        RAPID_FIRE          // Descărcare automată 3 bile (Bang-Bang)
    }
    private State globalState = State.SEARCH_AND_DESTROY;
    private final ElapsedTime stateTimer = new ElapsedTime();

    // ==========================================
    // 3. UTILITIES CUSTOM (Zero-Allocation)
    // ==========================================
    private FastSOTM sotm;
    private TurretAbsoluteController turret;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        // --- BULK CACHING MANUAL ---
        // Latență de citire redusă de la ~30ms la ~2ms.
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        sotm = new FastSOTM();
        turret = new TurretAbsoluteController();

        // Inițializare PedroPathing
        Robot.follower = Constants.createFollower(hardwareMap);
        Robot.follower.startTeleOpDrive(true);
        Robot.follower.setStartingPose(new Pose(0, 0, 0)); // UPDATE: Va prelua din Auto la meci

        // Siguranță mecanică
        robot.intake.intakeMid();
//        robot.intake.closeGate();

        telemetry.addLine("Sistem World Class Online (PedroPathing 2.0.6).");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- 1. ACTUALIZARE HARDWARE ---
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // --- 2. COMPENSAREA VOLTAJULUI ---
        currentVoltage = batteryVoltageSensor.getVoltage();
        vComp = NOMINAL_VOLTAGE / currentVoltage;

        // --- 3. ACTUALIZARE ODOMETRIE PEDRO PATHING ---
        Robot.follower.update();
        Pose pose = Robot.follower.getPose();

        // REPARAȚIA SUPREMĂ: Extragem X și Y din Viteză DIRECT, fără să mai importăm clasa Vector!
        // Metoda getXComponent() e standard în PedroPathing 2.x
        double vx = Robot.follower.getVelocity().getXComponent();
        double vy = Robot.follower.getVelocity().getYComponent();

        // --- 4. CONDUS WORLD CLASS (Vector Magnitude) ---
        handleProDriving();

        // --- 5. INTEGRARE MARROW FTC ---
        // Verificăm dacă suntem fizic cu robotul în interiorul zonei desenate

        boolean inShootingZone = true; // Fallback pentru teste

        // TODO: Conectați la senzorii voștri fizici
        boolean has3Balls = true;

        // --- 6. CREIERUL ROBOTULUI (FSM) ---
        switch (globalState) {

            case SEARCH_AND_DESTROY:
                robot.intake.pullBalls();
//                robot.intake.closeGate();

                // Oprim shooter-ul complet pentru salvarea bateriei
                robot.shooter.turretMotorLeft.setPower(0);
                robot.shooter.turretMotorRight.setPower(0);

                // Tureta pe centru absolut
                turret.trackToAngleRad(0);

                if (has3Balls || gamepad1.y) {
                    changeState(State.TRANSIT);
                }
                break;

            case TRANSIT:
                robot.intake.rest();
//                robot.intake.closeGate();
                robot.shooter.turretMotorLeft.setPower(0);
                robot.shooter.turretMotorRight.setPower(0);
                turret.trackToAngleRad(0);

                // MARROW CHECK: Putem intra în modul de tragere DOAR în Shooting Zone
                if (inShootingZone && gamepad1.left_trigger > 0.1) {
                    changeState(State.ENGAGING);
                } else if (gamepad1.a) {
                    changeState(State.SEARCH_AND_DESTROY); // Răzgândire
                }
                break;

            case ENGAGING:
                // SOTM O(1) - Unde va fi coșul?
                sotm.update(pose.getX(), pose.getY(), pose.getHeading(), vx, vy);

                // Tureta urmărește unghiul RELATIV față de corpul robotului
                turret.trackToAngleRad(sotm.targetTurretAngleRad - pose.getHeading());

                // Spooling Shooter
                robot.shooter.shooting = true;
                robot.shooter.targetVelocity = 2700; // Se poate calcula cu o ecuație pe baza sotm.virtualDistance
                robot.shooter.update();

                // Când shooter-ul e gata și apăsăm Right Trigger
                if (gamepad1.right_trigger > 0.1) {
                    changeState(State.RAPID_FIRE);
                }

                // FAILSAFE MARROW: Dacă inamicul ne împinge AFARĂ din zonă, opriti atacul!
                if (gamepad1.left_trigger <= 0.1 || !inShootingZone) {
                    changeState(State.TRANSIT);
                }
                break;

            case RAPID_FIRE:
                // SOTM rulează în continuare pentru imunitate la apărare
                sotm.update(pose.getX(), pose.getY(), pose.getHeading(), vx, vy);
                turret.trackToAngleRad(sotm.targetTurretAngleRad - pose.getHeading());

                long timeInState = (long) stateTimer.milliseconds();

                // Putere maximă (BANG-BANG), compensată de starea bateriei
                double maxPower = Math.min(1.0, 1.0 * vComp);

                // SECVENȚA BANG-BANG OVER-REV (3 BILE)
                if (timeInState < 50) {
                    robot.shooter.turretMotorLeft.setPower(maxPower);
                    robot.shooter.turretMotorRight.setPower(maxPower);
//                    robot.intake.openGate(); // Scapă Bila 1
                } else if (timeInState < 150) {
//                    robot.intake.closeGate(); // Retract
                    robot.shooter.turretMotorLeft.setPower(maxPower);
                    robot.shooter.turretMotorRight.setPower(maxPower);
                } else if (timeInState < 200) {
//                    robot.intake.openGate(); // Scapă Bila 2
                    robot.shooter.turretMotorLeft.setPower(maxPower);
                    robot.shooter.turretMotorRight.setPower(maxPower);
                } else if (timeInState < 300) {
//                    robot.intake.closeGate(); // Retract
                    robot.shooter.turretMotorLeft.setPower(maxPower);
                    robot.shooter.turretMotorRight.setPower(maxPower);
                } else if (timeInState < 350) {
//                    robot.intake.openGate(); // Scapă Bila 3
                    robot.shooter.turretMotorLeft.setPower(maxPower);
                    robot.shooter.turretMotorRight.setPower(maxPower);
                } else if (timeInState < 450) {
//                    robot.intake.closeGate(); // Finalizare
                } else {
                    changeState(State.SEARCH_AND_DESTROY); // Ciclul se reia automat
                }
                break;
        }

        telemetry.addData("State", globalState);
        telemetry.addData("Marrow Zone", inShootingZone ? "PRIME" : "OUTSIDE");
        telemetry.addData("Battery", String.format("%.1fV (Comp: %.2f)", currentVoltage, vComp));
        telemetry.addData("Turret Error (Deg)", Math.toDegrees(turret.lastError));
        telemetry.update();
    }

    private void changeState(State newState) {
        globalState = newState;
        stateTimer.reset();
        if (newState != State.ENGAGING && newState != State.RAPID_FIRE) {
            robot.shooter.shooting = false;
        }
    }

    /**
     * Vector Magnitude Scaling: Păstrează unghiurile perfecte de strafe
     */
    private void handleProDriving() {
        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double turn    = -gamepad1.right_stick_x;

        double r = Math.hypot(strafe, forward);
        double theta = Math.atan2(forward, strafe);

        double rScaled = Math.pow(r, 3);
        double turnScaled = Math.pow(turn, 3);

        double forwardScaled = rScaled * Math.sin(theta);
        double strafeScaled = rScaled * Math.cos(theta);

        Robot.follower.setTeleOpDrive(forwardScaled, strafeScaled, turnScaled, true);
    }

    // ====================================================================
    // CLASELE INTERNE PENTRU PROCESARE O(1)
    // ====================================================================

    /**
     * Calculator Balistic SOTM O(1)
     */
    public class FastSOTM {
        private final double GOAL_X = (Robot.alliance == Robot.Alliance.RED) ? 133.0 : 12.0;
        private final double GOAL_Y = 134.0;
        private final double BALL_SPEED = 250.0; // Inci/Secundă (Tunați empiric)

        public double targetTurretAngleRad = 0;
        public double virtualDistance = 0;

        public void update(double rx, double ry, double rh, double vx, double vy) {
            double dx = GOAL_X - rx;
            double dy = GOAL_Y - ry;
            double distSq = dx * dx + dy * dy;

            double vrSq = vx * vx + vy * vy;
            double dotProduct = dx * vx + dy * vy;

            double a = vrSq - (BALL_SPEED * BALL_SPEED);
            double b = -2.0 * dotProduct;
            double c = distSq;
            double delta = (b * b) - (4 * a * c);
            double timeOfFlight = -1.0;

            if (delta > 0) {
                double sqrtDelta = Math.sqrt(delta);
                double t1 = (-b + sqrtDelta) / (2 * a);
                double t2 = (-b - sqrtDelta) / (2 * a);
                if (t1 > 0 && t2 > 0) timeOfFlight = Math.min(t1, t2);
                else if (t1 > 0) timeOfFlight = t1;
                else if (t2 > 0) timeOfFlight = t2;
            }

            if (timeOfFlight <= 0) timeOfFlight = Math.sqrt(distSq) / BALL_SPEED;

            double virtualX = GOAL_X - (vx * timeOfFlight);
            double virtualY = GOAL_Y - (vy * timeOfFlight);

            virtualDistance = Math.hypot(virtualX - rx, virtualY - ry);
            targetTurretAngleRad = Math.atan2(virtualY - ry, virtualX - rx);
        }
    }

    /**
     * Turetă Absolută: CRServo + Encoder REV
     */
    public class TurretAbsoluteController {
        private CRServo turretServo;
        private DcMotorEx revEncoder;

        private final double TICKS_PER_REV = 8192.0; // REV Through Bore Encoder
        private final double TURRET_KP = 1.2;
        private final double TURRET_KD = 0.05;

        public double lastError = 0;
        private ElapsedTime timer = new ElapsedTime();

        public TurretAbsoluteController() {
            turretServo = hardwareMap.get(CRServo.class, "turretPivot");

            // Asigurați-vă că pe Control Hub ați pus "turretEncoder" la portul encoderului REV
            revEncoder = hardwareMap.get(DcMotorEx.class, "turretEncoder");
            revEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            revEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            timer.reset();
        }

        public void trackToAngleRad(double targetRelativeAngleRad) {
            double currentTicks = revEncoder.getCurrentPosition();
            double currentAngleRad = (currentTicks / TICKS_PER_REV) * (2 * Math.PI);

            double error = targetRelativeAngleRad - currentAngleRad;
            while (error > Math.PI) error -= 2 * Math.PI;
            while (error < -Math.PI) error += 2 * Math.PI;

            double dt = timer.seconds();
            timer.reset();
            double derivative = (dt > 0) ? (error - lastError) / dt : 0;
            lastError = error;

            double power = (error * TURRET_KP) + (derivative * TURRET_KD);

            if (Math.abs(error) < Math.toRadians(1.0)) {
                power = 0;
            }

            turretServo.setPower(power);
        }
    }
}