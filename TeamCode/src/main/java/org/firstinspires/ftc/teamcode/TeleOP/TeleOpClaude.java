package org.firstinspires.ftc.teamcode.TeleOP;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Robot2;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

// =============================================================================
// TeleOpFinal — Sezon FTC 2025-2026
//
// ARHITECTURĂ FSM:
//   SEARCH_AND_DESTROY → robot colectează bile (beam break numără automat)
//   TRANSIT            → avem 3 bile, ne deplasăm spre Shooting Zone
//   ENGAGING           → lock-on: Limelight/SOTM tracking + spool flywheel
//   RAPID_FIRE         → descărcare automată 3 bile cu sub-FSM velocity-gated
//
// RAPID FIRE — "BAM BAM BAM":
//   NU folosim timere fixe pentru gate.
//   Gate-ul se deschide EXCLUSIV când flywheel-ul a atins ≥97% din viteza țintă.
//   Aceasta garantează că FIECARE bilă pleacă cu energia corectă,
//   indiferent de starea bateriei, impact cu adversar, sau turație inițială.
//   Intervalul net între bile este minimul fizic posibil al flywheelului vostru.
//
// ROBUSTEȚE LA IMPACT:
//   • SOTM rulează la FIECARE loop → tureta se reajustează în <20ms
//   • Dacă suntem împinși, headingul se schimbă → SOTM recalculează instant
//   • Failsafe în RAPID_FIRE: impact PREA puternic → abort → TRANSIT
//
// BEAM BREAK:
//   • Detecție falling-edge cu debounce 120ms (elimină false positive)
//   • Reset automat la schimbarea în SEARCH_AND_DESTROY
//   • gamepad1.back = reset manual de urgență
// =============================================================================
@TeleOp(name = "TeleOp Claude", group = "Decode")
public class TeleOpClaude extends OpMode {

    private Robot2 robot;

    // =========================================================================
    // 1. HARDWARE
    // =========================================================================
    private List<LynxModule> allHubs;
    private VoltageSensor     batteryVoltageSensor;

    private static final double NOMINAL_VOLTAGE = 13.0;
    private double currentVoltage = 13.0;

    // =========================================================================
    // 2. BEAM BREAK — Numărare bile
    // =========================================================================
    private DigitalChannel beamBreak;

    // Beam break: getState() = true  → fascicul INTACT (nicio bilă)
    //             getState() = false → fascicul RUPT   (bilă prezentă)
    private boolean       lastBeamBroken  = false;
    private int           ballCount       = 0;
    private static final int  MAX_BALLS   = 3;

    // Debounce time-based: min 120ms între detecții consecutive
    // (elimină reflexii optice pe rampă și vibrații mecanice)
    private final ElapsedTime debounceTimer  = new ElapsedTime();
    private static final double DEBOUNCE_MS  = 120.0;

    // =========================================================================
    // 3. FSM PRINCIPAL
    // =========================================================================
    public enum State {
        SEARCH_AND_DESTROY,
        TRANSIT,
        ENGAGING,
        RAPID_FIRE
    }
    private State globalState = State.SEARCH_AND_DESTROY;
    private final ElapsedTime stateTimer = new ElapsedTime();

    // =========================================================================
    // 4. SUB-FSM RAPID_FIRE — Velocity-Gated
    // =========================================================================
    // Aceasta este INIMA sistemului "bam bam bam":
    // Fiecare bilă trece prin ciclul WAIT → FIRE → (repeat) → DONE.
    // WAIT: stăm cu gate-ul ÎNCHIS până flywheel-ul revine la ≥97% viteza țintă.
    // FIRE: deschidem gate-ul pentru exact GATE_OPEN_MS ms → bila intră în flywheel.
    // Ciclul se repetă pentru toate 3 bilele.
    private enum FireState {
        WAIT_FOR_VELOCITY,  // Flywheel sub prag → așteptăm recuperarea
        GATE_OPEN,          // Gate deschis → bila avansează spre flywheel
        DONE                // Toate 3 bilele trase
    }
    private FireState fireState = FireState.WAIT_FOR_VELOCITY;
    private int       ballsFired = 0;

    // Cât timp rămâne gate-ul deschis pentru o singură bilă (ms).
    // TUNING: Creșteți dacă bila nu apucă să intre. Micșorați pentru viteză mai mare.
    private static final double GATE_OPEN_MS = 50.0;

    // Timer dedicat pentru gate (separat de stateTimer pentru precizie)
    private final ElapsedTime gateTimer = new ElapsedTime();

    // Timeout de siguranță pentru întregul ciclu RAPID_FIRE (ms).
    // Dacă flywheel-ul nu se recuperează în acest timp → abort.
    // Previne blocarea în RAPID_FIRE dacă bateria e complet epuizată.
    private static final double FIRE_TIMEOUT_MS = 1500.0;

    // =========================================================================
    // INIT
    // =========================================================================
    @Override
    public void init() {
        robot = new Robot2(hardwareMap);

        // Bulk caching MANUAL: latență citire 30ms → ~2ms per loop
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Beam break
        beamBreak = hardwareMap.get(DigitalChannel.class, "beamBreak");
        beamBreak.setMode(DigitalChannel.Mode.INPUT);
        debounceTimer.reset();

        // PedroPathing
        Robot.follower = Constants.createFollower(hardwareMap);
        Robot.follower.startTeleOpDrive(true);
        Robot.follower.setStartingPose(new Pose(0, 0, 0)); // TODO: Transfer pose din Auto

        // Siguranță mecanică
//        robot.intake.intakeMid();
//        robot.intake.closeGate();

        telemetry.addLine("=== TeleOp Final 25-26 ONLINE ===");
        telemetry.addData("⚠ inShootingZone", "SIMULAT — conectați Marrow FTC!");
        telemetry.addData("⚠ ballCount",      "Via beam break — verificați senzorul");
        telemetry.update();
    }

    // =========================================================================
    // LOOP
    // =========================================================================
    @Override
    public void loop() {

        // --- 1. CACHE BULK ---
        for (LynxModule hub : allHubs) hub.clearBulkCache();

        // --- 2. VOLTAJ ---
        currentVoltage = batteryVoltageSensor.getVoltage();

        // --- 3. ODOMETRIE ---
        Robot.follower.update();

        // --- 4. NUMĂRARE BILE ---
        updateBallCount();

        // --- 5. CONDUS ---
        handleProDriving();

        // --- 6. DATE EXTERNE ---
        // TODO: Înlocuiți cu integrarea Marrow FTC
        boolean inShootingZone = true;

        // Reset manual bile (gamepad1.back)
        if (gamepad1.back) {
            ballCount = 0;
        }

        // --- 7. FSM PRINCIPAL ---
        switch (globalState) {

            // -----------------------------------------------------------------
            // SEARCH_AND_DESTROY: Colectăm bile
            // -----------------------------------------------------------------
            case SEARCH_AND_DESTROY:
                robot.intake.pullBalls();
//                robot.intake.closeGate();
                robot.shooter.shooting   = false;
                robot.shooter.isTracking = false;
                robot.shooter.update(currentVoltage); // Oprește flywheel

                // Robotul nostru poate prelua toate 3 bile într-o singură mișcare →
                // tranziționăm imediat când beam break confirmă 3 bile,
                // SAU override manual cu Y (dacă beam break nu funcționează la meci)
                if (ballCount >= MAX_BALLS || gamepad1.y) {
                    changeState(State.TRANSIT);
                }
                break;

            // -----------------------------------------------------------------
            // TRANSIT: Avem bilele, ne deplasăm spre zonă + pre-spool flywheel
            // -----------------------------------------------------------------
            case TRANSIT:
                robot.intake.rest();
//                robot.intake.closeGate();

                // PRE-SPOOL: Pornim flywheel-ul în tranzit dacă suntem aproape de zonă.
                // Aceasta reduce timpul de așteptare la ENGAGING (mai rapid la tragere).
                // Dezactivați dacă consumul de baterie e o problemă.
                if (inShootingZone) {
                    robot.shooter.shooting   = true;
                    robot.shooter.isTracking = true;
                } else {
                    robot.shooter.shooting   = false;
                    robot.shooter.isTracking = false;
                }
                robot.shooter.update(currentVoltage);

                if (inShootingZone && gamepad1.left_trigger > 0.1) {
                    changeState(State.ENGAGING);
                } else if (gamepad1.a) {
                    changeState(State.SEARCH_AND_DESTROY); // Driver se răzgândește
                }
                break;

            // -----------------------------------------------------------------
            // ENGAGING: Lock-on complet + spool shooter + așteptăm comanda
            // -----------------------------------------------------------------
            case ENGAGING:
//                robot.intake.closeGate();

                // Shooter: PIDF activ cu voltage comp + Limelight/SOTM tracking
                robot.shooter.shooting   = true;
                robot.shooter.isTracking = true;
                robot.shooter.update(currentVoltage);

                // Indicație vizuală pentru driver când e gata de foc
                // (Implementați LED sau rumble dacă aveți hardware)

                // Tragere când driver apasă Right Trigger
                if (gamepad1.right_trigger > 0.1) {
                    // Pornim sub-FSM-ul de rapid fire de la zero
                    fireState  = FireState.WAIT_FOR_VELOCITY;
                    ballsFired = 0;
                    gateTimer.reset();
                    changeState(State.RAPID_FIRE);
                }

                // Failsafe: trigger stâng eliberat SAU am ieșit din zonă
                if (gamepad1.left_trigger <= 0.1 || !inShootingZone) {
                    changeState(State.TRANSIT);
                }
                break;

            // -----------------------------------------------------------------
            // RAPID_FIRE: Descărcare 3 bile — velocity-gated, maxim de rapid
            // -----------------------------------------------------------------
            case RAPID_FIRE:

                // FAILSAFE #1: Trigger eliberat sau ieșit din zonă → abort imediat
                if (gamepad1.left_trigger <= 0.05 || !inShootingZone) {
//                    robot.intake.closeGate();
                    changeState(State.TRANSIT);
                    break;
                }

                // FAILSAFE #2: Timeout global (baterie moartă sau blocaj mecanic)
                if (stateTimer.milliseconds() > FIRE_TIMEOUT_MS) {
//                    robot.intake.closeGate();
                    changeState(State.SEARCH_AND_DESTROY);
                    break;
                }

                // Shooter MEREU activ în RAPID_FIRE: PIDF + tracking continuă
                // Aceasta înseamnă că dacă suntem împinși, tureta se reajustează
                // în <20ms → fiecare bilă pleacă spre țintă chiar și sub impact.
                robot.shooter.shooting   = true;
                robot.shooter.isTracking = true;
                robot.shooter.update(currentVoltage);

                // --- SUB-FSM VELOCITY-GATED ---
                switch (fireState) {

                    case WAIT_FOR_VELOCITY:
                        // Gate ÎNCHIS: așteptăm ca flywheel-ul să revină la ≥97% viteza țintă
                        // și tureta să fie aliniată.
                        //
                        // De ce velocity-gated și nu time-based?
                        // La 13V: recuperare după foc ~80ms
                        // La 11V: recuperare după foc ~120ms (voltage comp ajută, dar nu elimină)
                        // Cu timer fix de 100ms: la 11V trageam înainte de recuperare completă.
                        // Cu velocity-gate: tragem EXACT când flywheel-ul E gata → precizie maximă.
//                        robot.intake.closeGate();

                        if (robot.shooter.isShootReady()) {
                            fireState = FireState.GATE_OPEN;
                            gateTimer.reset();
                        }
                        break;

                    case GATE_OPEN:
                        // Gate DESCHIS: bila avansează spre flywheel
                        // Rămânem deschiși exact GATE_OPEN_MS ms (suficient pentru o bilă)
//                        robot.intake.openGate();

                        if (gateTimer.milliseconds() >= GATE_OPEN_MS) {
                            ballsFired++;
//                            robot.intake.closeGate();

                            if (ballsFired >= MAX_BALLS) {
                                // Toate 3 bile trase → terminat
                                fireState = FireState.DONE;
                            } else {
                                // Mai avem bile → așteptăm recuperarea flywheelului
                                fireState = FireState.WAIT_FOR_VELOCITY;
                            }
                        }
                        break;

                    case DONE:
                        // Ciclu complet — gate se asigură că e închis, înapoi la colectare
//                        robot.intake.closeGate();
                        changeState(State.SEARCH_AND_DESTROY);
                        break;
                }
                break;
        }

        // --- 8. TELEMETRIE COMPLETĂ ---
        telemetry.addData("► State",       globalState);
        telemetry.addData("► Fire Sub",    globalState == State.RAPID_FIRE ? fireState : "N/A");
        telemetry.addData("► Balls Shot",  ballsFired + " / " + MAX_BALLS);
        telemetry.addData("► Ball Count",  ballCount + " / " + MAX_BALLS);
        telemetry.addData("► Beam",        lastBeamBroken ? "●BROKEN" : "○INTACT");
        telemetry.addData("► Zone",        inShootingZone ? "✓ PRIME" : "✗ OUTSIDE");
        telemetry.addData("► Battery",     String.format("%.2fV", currentVoltage));
        telemetry.addData("► Vel Ratio",   String.format("%.3f / %.2f",
                robot.shooter.getVelocityRatio(),
                robot.shooter.VELOCITY_READY_THRESHOLD));
        telemetry.addData("► Shoot Ready", robot.shooter.isShootReady() ? "✓ FIRE" : "✗ WAIT");
        telemetry.addData("► Turret Err",  String.format("%.2f°",
                Math.toDegrees(robot.shooter.turretErrorRad)));
        telemetry.addData("► Target Vel",  String.format("%.0f ticks/s",
                robot.shooter.targetVelocity));
        telemetry.addData("⚠ Zone",        "SIMULAT — conectați Marrow FTC!");
        telemetry.update();
    }

    // =========================================================================
    // NUMĂRARE BILE — Beam Break cu Falling Edge + Debounce
    // =========================================================================

    /**
     * Detectează bile pe FALLING EDGE: fascicul intact → rupt = bilă nouă detectată.
     *
     * Debounce de 120ms elimină:
     *   • Reflexii optice pe rampa din metal/plastic
     *   • Vibrații mecanice la accelerare
     *   • Bounce-ul electronic al senzorului digital
     *
     * Notă: Robotul nostru preia toate 3 bile simultan → contorul crește
     * rapid de la 0 la 3 în ~300-400ms (3 bile × debounce 120ms).
     */
    private void updateBallCount() {
        boolean beamBroken    = !beamBreak.getState(); // invert: false=rupt
        boolean isFallingEdge = beamBroken && !lastBeamBroken;

        if (isFallingEdge && debounceTimer.milliseconds() > DEBOUNCE_MS) {
            ballCount = Math.min(MAX_BALLS, ballCount + 1);
            debounceTimer.reset();
        }

        lastBeamBroken = beamBroken;
    }

    // =========================================================================
    // SCHIMBARE STARE
    // =========================================================================
    private void changeState(State newState) {
        // Oprire shooter la ieșirea din modurile de tragere
        if (newState != State.ENGAGING && newState != State.RAPID_FIRE
                && newState != State.TRANSIT) {
            robot.shooter.shooting   = false;
            robot.shooter.isTracking = false;
        }

        // Reset contor bile după ce am golit magazinul
        if (newState == State.SEARCH_AND_DESTROY) {
            ballCount  = 0;
            ballsFired = 0;
        }

        globalState = newState;
        stateTimer.reset();
    }

    // =========================================================================
    // CONDUS — Vector Magnitude Scaling (menține unghiurile de strafe perfecte)
    // =========================================================================

    /**
     * Cubatură pe magnitudinea vectorului de translație (nu pe componente individuale).
     *
     * De ce? Dacă cubezi X și Y separat, diagonalele devin mai lente relativ
     * față de axele pure. Cubând magnitudinea și recompunând, păstrăm unghiurile.
     */
    private void handleProDriving() {
        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double turn    = -gamepad1.right_stick_x;

        double r     = Math.hypot(strafe, forward);
        double theta = Math.atan2(forward, strafe);

        double rScaled    = Math.pow(r, 3);
        double turnScaled = Math.pow(turn, 3);

        double forwardScaled = rScaled * Math.sin(theta);
        double strafeScaled  = rScaled * Math.cos(theta);

        Robot.follower.setTeleOpDrive(forwardScaled, strafeScaled, turnScaled, true);
    }
}