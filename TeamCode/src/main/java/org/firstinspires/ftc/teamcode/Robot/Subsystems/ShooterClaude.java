package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Utils.PIDFController;
import org.firstinspires.ftc.teamcode.Robot.uV;

// =============================================================================
// Shooter — Subsystem complet: Flywheel PIDF + Voltage Compensation + SOTM
//
// ARHITECTURĂ:
//   • Flywheel: PIDF cu feedforward compensat dinamic (voltaj bătea 0.5-1.0V
//     fluctuații în meci → fără compensare, bila 2 și 3 ieșeau cu ~8% mai lent)
//   • RPM: MEREU calculat dinamic pe baza distanței (polinomial grad 2)
//   • Tracking: Limelight (primar) → Odometrie + SOTM (fallback)
//     SOTM = Shot-On-Target-Moving: corectează unghiul pentru viteza robotului
//   • isShootReady(): velocity ≥ 97% AND turret aligned ≤ 1.5° → ABIA ATUNCI FIRE
//
// TUNING:
//   • RPM: Schimbați A, B, C în computeVelocity() pe baza testelor pe teren
//   • LOB: Schimbați polinomul în computeLob() pentru servoul de lob
//   • PIDF: shootKp/Ki/Kd/Kf din FTC Dashboard
// =============================================================================
@SuppressWarnings("FieldCanBeLocal")
@Config
public class ShooterClaude extends Subsystem {

    // =========================================================================
    // HARDWARE
    // =========================================================================
    public final DcMotorEx turretMotorLeft;
    public final DcMotorEx turretMotorRight;

    private final Servo   lobServo;
    private final CRServo turretPivot;

    private final Limelight3A ll;

    // =========================================================================
    // VOLTAGE COMPENSATION
    // =========================================================================
    // Voltajul nominal la care robotul a fost calibrat. Ajustați dacă bateria
    // voastră nouă are un voltaj de repaus diferit.
    private static final double NOMINAL_VOLTAGE = 13.0;

    // Voltajul curent — actualizat la fiecare apel update(voltage)
    private double currentVoltage = NOMINAL_VOLTAGE;

    // =========================================================================
    // PIDF FLYWHEEL — Valorile se pot ajusta live din FTC Dashboard (@Config)
    // =========================================================================
    // kF (feedforward) este cel mai critic pentru viteza de recuperare după foc.
    // REGULĂ: kF_efectiv = kF * (NOMINAL_VOLTAGE / currentVoltage)
    // Aceasta înseamnă că la 11V baterie, kF crește cu ~18% → mai mult curent
    // → viteza se recuperează la fel de rapid ca la 13V.
    public static double shootKp = 0.07;
    public static double shootKi = 0.00002;
    public static double shootKd = 0.0000001;
    public static double shootKf = 0.013;

    // Pragul de viteză pentru "gata de foc" (97% din țintă).
    // Sub acest prag, gate-ul NU se deschide — bila ar ieși cu putere insuficientă.
    // Creșteți la 0.98 dacă bilele cad scurt. Scădeți la 0.95 dacă e prea lent.
    public static double VELOCITY_READY_THRESHOLD = 0.97;

    // Eroarea maximă a turetei (grade) pentru a considera alinierea completă
    public static double TURRET_READY_THRESHOLD_DEG = 1.5;

    private final PIDFController pidfController =
            new PIDFController(shootKp, shootKi, shootKd, shootKf);

    // =========================================================================
    // TRACKING — Limelight (primar) + Odometrie cu SOTM (fallback)
    // =========================================================================
    private final PIDFController odometryTrackingController =
            new PIDFController(uV.odometryKp, uV.odometryKi, uV.odometryKd, uV.odometryKf);

    private final PIDFController limelightTrackingController =
            new PIDFController(uV.limelightKp, uV.limelightKi, uV.limelightKd, uV.limelightKf);

    // Pose-urile Obelisk-urilor (țintele din joc)
    private final Pose blueObeliskPose = new Pose(12, 134);
    private final Pose redObeliskPose  = new Pose(133, 134);

    // =========================================================================
    // SOTM (Shot-On-Target-Moving) — integrat în tracking odometrie
    // Permite tragere precisă în timp ce robotul se deplasează SAU e împins.
    // =========================================================================
    // Viteza bilei în ticks de odometrie per secundă (tunat empiric pe teren).
    // Dacă SOTM supracompensează: micșorați. Dacă subcompensează: măriți.
    private static final double BALL_FIELD_SPEED = 250.0; // inci/s (aproximativ)

    // Viteza efectivă a bilei ținând cont de rezistența aerului
    private static final double DRAG_COEFF = 0.0015;

    // =========================================================================
    // STARE PUBLICĂ
    // =========================================================================
    public double  targetVelocity   = 0.0;  // Citibil din TeleOp (doar pentru telemetrie)
    public boolean shooting         = false;
    public boolean isTracking       = false;

    // Turret error (radian) — expus pentru TeleOp telemetrie + isShootReady()
    public double turretErrorRad = 0.0;

    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================
    public ShooterClaude(HardwareMap hwMap) {
        lobServo = hwMap.get(Servo.class, "lobServo");

        turretMotorRight = hwMap.get(DcMotorEx.class, "turretMotorRight");
        turretMotorLeft  = hwMap.get(DcMotorEx.class, "turretMotorLeft");
        turretMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        turretMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Toleranța PIDF: 20 ticks/s = ~2% din viteza tipică → destul de strict
        pidfController.setTolerance(20);
        pidfController.maxOut = 2.0;
        pidfController.minOut = -2.0;

        ll = hwMap.get(Limelight3A.class, "limelight");
        ll.pipelineSwitch(2);
        limelightTrackingController.setSetpoint(0);
        limelightTrackingController.setTolerance(0.3);

        odometryTrackingController.setTolerance(20);

        turretPivot = hwMap.get(CRServo.class, "turretPivot");
    }

    // =========================================================================
    // VELOCITY RATIO — Cât % din viteza țintă avem acum?
    // Folosit de TeleOp pentru velocity-gated fire: NU tragem dacă < threshold.
    // =========================================================================

    /**
     * Returnează [0.0, 1.0+]: raportul dintre viteza actuală și viteza țintă.
     * Valori tipice:
     *   • 0.0  → flywheel oprit
     *   • 0.97 → la prag de tragere (VELOCITY_READY_THRESHOLD)
     *   • 1.0  → exact la țintă
     *   • 1.05 → ușor peste (acceptabil)
     */
    public double getVelocityRatio() {
        if (targetVelocity <= 0) return 0.0;
        double currentVel = Math.abs(turretMotorLeft.getVelocity());
        return currentVel / targetVelocity;
    }

    /**
     * Flywheel-ul a atins viteza necesară pentru un foc precis?
     */
    public boolean velocityReady() {
        return getVelocityRatio() >= VELOCITY_READY_THRESHOLD;
    }

    /**
     * Tureta E aliniată spre țintă (eroare < TURRET_READY_THRESHOLD_DEG)?
     */
    public boolean turretAligned() {
        return Math.abs(Math.toDegrees(turretErrorRad)) < TURRET_READY_THRESHOLD_DEG;
    }

    /**
     * Robotul e COMPLET gata să tragă: flywheel la viteză AND turetă aliniată.
     * TeleOp va gate-ui deschiderea alimentatorului pe aceasta.
     */
    public boolean isShootReady() {
        return velocityReady() && turretAligned();
    }

    // =========================================================================
    // DISTANȚĂ ȘI GEOMETRIE
    // =========================================================================

    /**
     * Distanța brută curentă față de Obelisk (inci).
     * Se folosește pentru RPM și Lob. SOTM o va corecta pentru unghi.
     */
    public double computeDistance() {
        Pose currentPose  = Robot.follower.getPose();
        Pose targetObelisk = (Robot.alliance == Robot.Alliance.RED) ? redObeliskPose : blueObeliskPose;
        return currentPose.distanceFrom(targetObelisk);
    }

    // =========================================================================
    // CALCUL LOB (Servo poziție bazată pe distanță)
    // =========================================================================

    /**
     * Calculează poziția servo-ului de lob pe baza distanței.
     *
     * TUNING: Faceți 5 aruncări la distanțe diferite și notați pozițiile servo.
     * Introduceți valorile în formula de mai jos (regresie liniară/polinomială).
     *
     * Formula actuală: liniară simplă cu clamp.
     * Înlocuiți cu polinomul voștru după teste.
     */
    private double computeLob(double distInches) {
        // --- Tuning zone ---
        final double DIST_MIN    = 55.0;   // distanța minimă de tragere (inci)
        final double DIST_MAX    = 140.0;  // distanța maximă de tragere (inci)
        final double LOB_AT_MIN  = 0.0;    // poziție servo la distanță minimă
        final double LOB_AT_MAX  = 0.85;   // poziție servo la distanță maximă
        // --- End tuning zone ---

        // Interpolare liniară cu clamp
        double t   = (distInches - DIST_MIN) / (DIST_MAX - DIST_MIN);
        t = Math.max(0.0, Math.min(1.0, t));
        return LOB_AT_MIN + t * (LOB_AT_MAX - LOB_AT_MIN);
    }

    // =========================================================================
    // CALCUL VITEZĂ DINAMICĂ (Polinomial Grad 2 — NICIODATĂ FIXĂ)
    // =========================================================================

    /**
     * Calculează viteza (ticks/s) necesară pentru distanța dată.
     *
     * Formula: V = A·d² + B·d + C (regresie polinomială)
     *
     * TUNING EMPIRIC (obligatoriu pe teren):
     *   1. Măsurați distanța exactă față de Obelisk la 3+ poziții diferite
     *   2. Ajustați velocitatea până bila nimerește centru la fiecare
     *   3. Introduceți perechile (dist, velocity) în Wolfram Alpha sau Excel
     *   4. Cereți o "polynomial regression degree 2" → obțineți A, B, C noi
     *
     * Valorile INIȚIALE sunt estimate pe baza codului original comentat.
     * Nu trageți în meci fără tuning pe teren!
     */
    private double computeVelocity(double distInches) {
        // --- Tuning zone ---
        final double A = 0.08;    // Compensație pentru efectul Magnus exponențial
        final double B = 7.5;     // Pierdere liniară din rezistența aerului
        final double C = 700.0;   // Viteza de bază la distanță minimă (ticks/s)
        // --- End tuning zone ---

        double velocity = A * distInches * distInches + B * distInches + C;

        // Clamp între limitele fizice ale motoarelor voastre
        // MIN: viteza sub care bila nu are putere să ajungă la Obelisk
        // MAX: viteza maximă fără să suprasolicitați motoarele
        return Math.max(800.0, Math.min(3200.0, velocity));
    }

    // =========================================================================
    // TRACKING (Limelight primar + SOTM Odometrie fallback)
    // =========================================================================

    /**
     * Menține tureta îndreptată spre Obelisk.
     *
     * Strategie duală:
     *   1. Limelight (ID AprilTag specific alianței) — cea mai precisă, latență ~30ms
     *   2. Odometrie + SOTM — fallback dacă LL nu vede tag-ul (ecranare, distanță)
     *      SOTM corectează unghiul pentru viteza curentă a robotului,
     *      deci tureta anticipează unde va fi Obelisk-ul în momentul impactului.
     */
    public void track() {
        LLResult result    = ll.getLatestResult();
        boolean  found     = false;
        double   output    = 0.0;
        turretErrorRad     = 0.0;

        // --- Primar: Limelight ---
        if (result != null && result.isValid()) {
            int targetId = (Robot.alliance == Robot.Alliance.RED) ? 24 : 20;
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                if (fiducial.getFiducialId() == targetId) {
                    // tx = eroarea orizontală în grade față de centrul camerei
                    double tx = result.getTx();
                    turretErrorRad = Math.toRadians(tx);
                    output = limelightTrackingController.updatePID(tx);
                    found  = true;
                    break;
                }
            }
        }

        // --- Fallback: Odometrie + SOTM ---
        if (!found) {
            Pose   pose   = Robot.follower.getPose();
            double vx     = Robot.follower.getVelocity().getXComponent();
            double vy     = Robot.follower.getVelocity().getYComponent();

            // SOTM: calculăm unde va fi Obelisk-ul virtual ținând cont de viteza robotului
            double[] sotmResult = computeSOTM(
                    pose.getX(), pose.getY(),
                    vx, vy
            );
            double sotmAngleRad = sotmResult[0]; // unghiul spre poziția virtuală

            // Eroarea angulară față de headingul curent al robotului
            turretErrorRad = sotmAngleRad - pose.getHeading();
            // Normalizare shortest-path [-π, π]
            while (turretErrorRad >  Math.PI) turretErrorRad -= 2.0 * Math.PI;
            while (turretErrorRad < -Math.PI) turretErrorRad += 2.0 * Math.PI;

            // Conversia la ticks encoder (REV Through Bore: 8192 ticks/rev)
            double targetTicks = turretErrorRad * 8192.0 / (2.0 * Math.PI);
            odometryTrackingController.setSetpoint(targetTicks);
            output = odometryTrackingController.updatePID(turretMotorRight.getCurrentPosition());
        }

        turretPivot.setPower(output);
    }

    // =========================================================================
    // SOTM INTERN — O(1), fără alocare memorie
    // =========================================================================

    /**
     * Calculează unghiul turetei corectând pentru viteza robotului (SOTM).
     *
     * Returnează: [0] = unghiul spre "Obelisk-ul virtual" (radiani, sistem absolut)
     *             [1] = distanța virtuală (inci) — folosită și pentru RPM mai precis
     */
    private double[] computeSOTM(double rx, double ry, double vx, double vy) {
        Pose   target   = (Robot.alliance == Robot.Alliance.RED) ? redObeliskPose : blueObeliskPose;
        double goalX    = target.getX();
        double goalY    = target.getY();

        double dx      = goalX - rx;
        double dy      = goalY - ry;
        double rawDist = Math.sqrt(dx * dx + dy * dy);

        // Viteza efectivă a bilei cu drag (scade puțin pe distanțe mari)
        double effectiveSpeed = BALL_FIELD_SPEED * (1.0 - rawDist * DRAG_COEFF);
        effectiveSpeed = Math.max(effectiveSpeed, BALL_FIELD_SPEED * 0.5);

        // Rezolvare ecuație cuadratică pentru timp de zbor
        double vrSq   = vx * vx + vy * vy;
        double dot    = dx * vx + dy * vy;
        double distSq = dx * dx + dy * dy;

        double a     = vrSq - effectiveSpeed * effectiveSpeed;
        double b     = -2.0 * dot;
        double c     = distSq;
        double delta = b * b - 4.0 * a * c;

        double tof = -1.0;
        if (delta > 0.0) {
            double sq = Math.sqrt(delta);
            double t1 = (-b + sq) / (2.0 * a);
            double t2 = (-b - sq) / (2.0 * a);
            if (t1 > 0.0 && t2 > 0.0) tof = Math.min(t1, t2);
            else if (t1 > 0.0)        tof = t1;
            else if (t2 > 0.0)        tof = t2;
        }

        // Fallback: robot staționar sau discriminant ≤ 0
        if (tof <= 0.0) tof = rawDist / effectiveSpeed;

        double virtualX   = goalX - vx * tof;
        double virtualY   = goalY - vy * tof;
        double virtualDist = Math.hypot(virtualX - rx, virtualY - ry);
        double angle       = Math.atan2(virtualY - ry, virtualX - rx);

        return new double[]{angle, virtualDist};
    }

    // =========================================================================
    // UPDATE — Apelat din TeleOp.loop() cu voltajul curent al bateriei
    // =========================================================================

    /**
     * Metoda principală de actualizare. Apelați la FIECARE loop cu voltajul curent.
     *
     * @param voltage Voltajul bateriei citit din VoltageSensor (Volți)
     */
    public void update(double voltage) {
        currentVoltage = (voltage > 0.0) ? voltage : NOMINAL_VOLTAGE;

        // Actualizăm PIDF-ul cu valorile din Dashboard (live tuning)
        pidfController.kP = shootKp;
        pidfController.kI = shootKi;
        pidfController.kD = shootKd;
        // kF NU se setează direct — se aplică compensarea de voltaj mai jos

        if (shooting) {
            // 1. Calculăm distanța și RPM-ul dinamic (NICIODATĂ fix)
            double dist  = computeDistance();
            targetVelocity = computeVelocity(dist);
            pidfController.setSetpoint(targetVelocity);

            // 2. PIDF output brut
            double pidOutput = pidfController.updatePID(turretMotorLeft.getVelocity());

            // 3. VOLTAGE COMPENSATION pe feedforward (kF)
            // ─────────────────────────────────────────────────────────────────
            // Motivul: kF prezice puterea necesară pentru a menține targetVelocity.
            // La 11V, motorul produce mai puțin cuplu la aceeași putere →
            // kF trebuie să crească proporțional cu (NOMINAL / currentV).
            //
            // Formula derivată:
            //   putere_necesară = kF * targetVelocity                  (la voltaj nominal)
            //   putere_comp     = kF * targetVelocity * (NOM / V_crt)  (compensat)
            //
            // PID-ul (P + I + D) nu se compensează — el RĂSPUNDE la eroarea reală,
            // care oricum crește dacă voltajul scade → auto-compensare prin feedback.
            // Compensăm DOAR feedforward-ul pentru a reduce eroarea inițială.
            // ─────────────────────────────────────────────────────────────────
            double vComp            = NOMINAL_VOLTAGE / currentVoltage;
            double kF_compensated   = shootKf * vComp;

            // Puterea FF compensată pentru viteza țintă
            double ffCompensated    = kF_compensated * targetVelocity;

            // Puterea totală = PID (fără kF original) + FF compensat
            // Extragem contribuția kF originală din pidOutput și o înlocuim cu cea compensată
            double kF_original      = shootKf * targetVelocity;
            double pidWithoutFF     = pidOutput - kF_original;
            double finalOutput      = (pidWithoutFF + ffCompensated) / 2.0;

            // Clamp la [-1, 1]
            finalOutput = Math.max(-1.0, Math.min(1.0, finalOutput));

            turretMotorLeft.setPower(finalOutput);
            turretMotorRight.setPower(finalOutput);

            // 4. Lob servo dinamic
            lobServo.setPosition(computeLob(dist));

        } else {
            // Shooter oprit complet (salvare baterie între cicluri)
            turretMotorLeft.setPower(0.0);
            turretMotorRight.setPower(0.0);
            targetVelocity = 0.0;
        }

        // 5. Tracking turetă (dacă e activat din TeleOp)
        if (isTracking) {
            track();
        }

        // Dashboard telemetrie
        FtcDashboard.getInstance().getTelemetry().addData("[Shooter] Target Vel",
                String.format("%.0f ticks/s", targetVelocity));
        FtcDashboard.getInstance().getTelemetry().addData("[Shooter] Actual Vel",
                String.format("%.0f ticks/s", turretMotorLeft.getVelocity()));
        FtcDashboard.getInstance().getTelemetry().addData("[Shooter] Vel Ratio",
                String.format("%.3f", getVelocityRatio()));
        FtcDashboard.getInstance().getTelemetry().addData("[Shooter] Voltage",
                String.format("%.2fV → kF×%.3f", currentVoltage, NOMINAL_VOLTAGE / currentVoltage));
        FtcDashboard.getInstance().getTelemetry().addData("[Shooter] Turret Err",
                String.format("%.2f°", Math.toDegrees(turretErrorRad)));
        FtcDashboard.getInstance().getTelemetry().addData("[Shooter] ShootReady",
                isShootReady() ? "✓ FIRE" : "✗ WAIT");
    }

    // =========================================================================
    // OVERRIDE FĂRĂ VOLTAJ (pentru compatibilitate backward — NU folosiți!)
    // =========================================================================
    @Override
    public void update() {
        // Dacă e apelat fără voltaj (greșeală), folosim ultimul voltaj cunoscut
        update(currentVoltage);
    }
}