package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * ============================================================================
 *  ΚΟΥΜΠΙΑ (gamepad1):
 *    Αριστερό stick  -> κίνηση (μπρος/πίσω/strafe)
 *    Δεξί stick (x)  -> στροφή
 *    START           -> reset yaw / re-arm μετά από kill switch
 *    LB -> SHOOTER LEFT  ON/OFF   (κλείνει σε SHOOTER_TARGET_RPM)
 *    RB -> SHOOTER RIGHT ON/OFF   (κλείνει σε SHOOTER_TARGET_RPM)
 *    LS -> HEADING HOLD ON/OFF
 *    B  -> SLOW MODE ON/OFF
 *    Y  -> FIELD CENTRIC / ROBOT CENTRIC
 *    X  -> INTAKE ON/OFF
 *    BACK -> KILL SWITCH (όλα OFF)
 *    D-PAD -> UP: servo1, DOWN: servo2, RIGHT: servo3, LEFT: servo4
 * ============================================================================
 */

@TeleOp(name = "Grobo-DriveTrain_Shooter_Servo")
public class DriveTrainServoShooterNEW extends LinearOpMode {

    // =========================================================================
    // ΡΥΘΜΙΣΕΙΣ - άλλαξτε ΜΟΝΟ εδώ
    // =========================================================================


    private static final double SHOOTER_TICKS_PER_REV = 28;

    // Θεωρητικό no-load max RPM του motor.
    private static final double SHOOTER_MAX_RPM = 6000;

    // Target RPM - σε αυτό κουρνιάει ο shooter όταν είναι ON.
    private static final double SHOOTER_TARGET_RPM = 4000;

    // PID

    private static final double SHOOTER_KP = 0.00030;
    private static final double SHOOTER_KI = 0.00030;
    private static final double SHOOTER_KD = 0.00001;

    // Integral clamp. Αν αυξήσεις kI αυξήσεις και το όριο.
    private static final double SHOOTER_INTEGRAL_LIMIT = 1500;

    // READY όταν |current - target| <= SHOOTER_READY_ERROR_RPM.
    private static final double SHOOTER_READY_ERROR_RPM = 120.0;

    // true -> peak RPM, peak ρεύμα, spin-up time στο telemetry (left shooter).
    private static final boolean SHOOTER_DEBUG_TELEMETRY = false;

    // ---------- INTAKE ----------
    private static final double INTAKE_TICKS_PER_REV = 28;

    // ---------- DRIVE ----------
    // Deadband (0..1) στα sticks: κάτω από αυτό, input = 0 (κανένα drift).
    private static final double STICK_DEADBAND = 0.05;

    // Exponential stick curve: 1.0 = linear, 1.5-1.7 = precision feel, 2.0 = squared.
    private static final double STICK_EXPONENT = 1.6;

    // Slow mode (μπρος/πίσω/strafe και στροφή).
    private static final double SLOW_DRIVE = 0.30;
    private static final double SLOW_TURN = 0.30;

    // HEADING HOLD:
    private static final boolean HEADING_HOLD_DEFAULT = true;
    private static final double HEADING_HOLD_KP = 0.010;      // power / deg σφάλματος
    private static final double HEADING_HOLD_MAX_POWER = 0.45;
    private static final double HEADING_HOLD_DEADBAND_DEG = 1.0;
    // Αν κρατάει ανάποδα (σε σχέση με stick), βάλε -1.
    private static final double HEADING_HOLD_DIRECTION = 1.0;

    // Slew rate limit - ΜΕΓΙΣΤΗ αλλαγή ανά δευτερόλεπτο. Μεγαλύτερο = πιο άμεσα.
    // 6.0/sec = 0 σε 1.0 power σε ~167 ms. Αν νιώσεις "lag", αύξησέ το.
    private static final double DRIVE_SLEW_RATE_PER_SEC = 6.0;
    private static final double TURN_SLEW_RATE_PER_SEC = 8.0;

    // ---------- SERVOS ----------
    private static final double SERVO_REST = 0.1;
    private static final double SERVO_ACTIVE = 0.9;


    private static class VelocityPIDF {

        private final double kP, kI, kD, kF;
        private final double integralLimit;

        private double integral = 0;
        private double lastError = 0;
        private double smoothedDerivative = 0;
        private boolean firstRun = true;

        VelocityPIDF(double kP, double kI, double kD, double kF, double integralLimit) {
            this.kP = kP; this.kI = kI; this.kD = kD;
            this.kF = kF;
            this.integralLimit = integralLimit;
        }

        void reset() {
            integral = 0;
            lastError = 0;
            smoothedDerivative = 0;
            firstRun = true;
        }

        double calculate(double targetTicksPerSec, double currentTicksPerSec, double dt) {
            if (!(dt > 0) || dt > 0.5) dt = 0.02; // guard για loop hiccups

            double error = targetTicksPerSec - currentTicksPerSec;

            // Integral με anti-windup clamp.
            integral = clamp(integral + error * dt, -integralLimit, integralLimit);

            // Derivative με low-pass smoothing. Πρώτο loop: derivative = 0.
            double derivative = 0;
            if (!firstRun) {
                double raw = (error - lastError) / dt;
                smoothedDerivative = 0.5 * raw + 0.5 * smoothedDerivative;
                derivative = smoothedDerivative;
            } else {
                firstRun = false;
            }
            lastError = error;

            double output = kF * targetTicksPerSec
                    + kP * error
                    + kI * integral
                    + kD * derivative;

            return clamp(output, 0.0, 1.0);
        }

        double getIntegral() { return integral; }
    }

    // =========================================================================
    // MAIN OPMODE
    // =========================================================================

    @Override
    public void runOpMode() {


        boolean fieldCentric = false;
        boolean lastY = false, lastStart = false;
        boolean intakeOn = false, lastX = false;
        boolean shooterLeftOn = false, lastLB = false;
        boolean shooterRightOn = false, lastRB = false;
        boolean slowMode = false, lastB = false;
        boolean headingHold = HEADING_HOLD_DEFAULT, lastLS = false;
        boolean killSwitch = false, lastBack = false;
        boolean servo1Active = false, servo2Active = false;
        boolean servo3Active = false, servo4Active = false;
        boolean lastUp = false, lastDown = false, lastRight = false, lastLeft = false;

        // Παρακολούθηση shooter left (peak / spin-up)
        ElapsedTime shooterLeftTimer = new ElapsedTime();
        double shooterLeftPeakRpm = 0;
        double shooterLeftPeakCurrent = 0;
        double shooterLeftSpinUpTime = -1;

        // READY / rumble flags (ένα blip ανά shooter, μιά φορά)
        boolean shooterLeftReady = false;
        boolean shooterLeftRumbleDone = false;
        boolean shooterRightReady = false;
        boolean shooterRightRumbleDone = false;

        // Loop timing
        ElapsedTime loopTimer = new ElapsedTime();
        double lastLoopTime = 0;

        // Telemetry throttle (update κάθε 3 loops)
        int telemetryCounter = 0;

        // Slew tracked values
        double lastStrafeCmd = 0;
        double lastForwardCmd = 0;
        double lastTurnCmd = 0;

        // Heading-hold reference
        double holdHeadingDeg = 0;

        // ---------------- Hardware ----------------
        Motor frontLeft = new Motor(hardwareMap, "leftFMotor");
        Motor frontRight = new Motor(hardwareMap, "rightFMotor");
        Motor backLeft = new Motor(hardwareMap, "leftBMotor");
        Motor backRight = new Motor(hardwareMap, "rightBMotor");

        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0);

        DcMotorEx shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        DcMotorEx shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        // ΠΡΟΣΟΧΗ: και τα δύο REVERSE. Αν τα motors είναι αντικριστά (καθρέφτης),
        // το ένα πρέπει να είναι FORWARD. Όπως πριν - έλεγξέ το στο ρομπότ.
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // RUN_WITHOUT_ENCODER = το power πάει κατευθείαν στο motor. Το PID
        // loop γίνεται εδώ στο OpMode, με feedback από getVelocity().
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setPower(0);
        shooterRight.setPower(0);

        Servo servo1 = hardwareMap.get(Servo.class, "servo1");
        Servo servo2 = hardwareMap.get(Servo.class, "servo2");
        Servo servo3 = hardwareMap.get(Servo.class, "servo3");
        Servo servo4 = hardwareMap.get(Servo.class, "servo4");

        // Αν κάποιο servo κουνιέται ανάποδα, βάλε REVERSE εδώ
        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.FORWARD);
        servo3.setDirection(Servo.Direction.FORWARD);
        servo4.setDirection(Servo.Direction.FORWARD);

        servo1.setPosition(SERVO_REST);
        servo2.setPosition(SERVO_REST);
        servo3.setPosition(SERVO_REST);
        servo4.setPosition(SERVO_REST);

        MecanumDrive drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        // ---------------- PIDF controllers ----------------
        // Feed-forward scale: 1 / no-load ticks-per-sec
        double shooterMaxTicksPerSec = SHOOTER_MAX_RPM / 60.0 * SHOOTER_TICKS_PER_REV;
        double kFeedForward = 1.0 / shooterMaxTicksPerSec;

        VelocityPIDF shooterLeftPid = new VelocityPIDF(
                SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, kFeedForward, SHOOTER_INTEGRAL_LIMIT);
        VelocityPIDF shooterRightPid = new VelocityPIDF(
                SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, kFeedForward, SHOOTER_INTEGRAL_LIMIT);

        double shooterTargetTicksPerSec = SHOOTER_TARGET_RPM / 60.0 * SHOOTER_TICKS_PER_REV;

        // ---------------- Ready ----------------
        telemetry.addData("Status", "Ready (PIDF edition)");
        telemetry.addData("Drive Mode", "ROBOT CENTRIC");
        telemetry.addData("Shooter Target", "%.0f RPM", SHOOTER_TARGET_RPM);
        telemetry.addData("Shooter PIDF", "kF=%.6f kP=%.5f kI=%.5f kD=%.6f",
                kFeedForward, SHOOTER_KP, SHOOTER_KI, SHOOTER_KD);
        telemetry.addLine("BACK = kill switch | START = re-arm/reset yaw | LS = heading hold");
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        // ========================= MAIN LOOP =========================
        while (opModeIsActive()) {

            // --- Loop dt (για slew + PIDF): fallback 20 ms, clamp για safety ---
            double nowSec = loopTimer.seconds();
            double dt = nowSec - lastLoopTime;
            lastLoopTime = nowSec;
            if (!(dt > 0) || dt > 0.5) dt = 0.02;

            // --- KILL SWITCH (BACK) / RE-ARM (START) ---
            if (gamepad1.back && !lastBack) {
                killSwitch = true;
                intakeOn = false;
                shooterLeftOn = false;
                shooterRightOn = false;
                servo1Active = false;
                servo2Active = false;
                servo3Active = false;
                servo4Active = false;
                shooterLeftPid.reset();
                shooterRightPid.reset();
            }
            lastBack = gamepad1.back;

            if (gamepad1.start && !lastStart) {
                imu.resetYaw();
                holdHeadingDeg = 0;
                killSwitch = false; // re-arm
            }
            lastStart = gamepad1.start;

            if (!killSwitch) {
                // --- FIELD / ROBOT CENTRIC (Y) ---
                if (gamepad1.y && !lastY) {
                    fieldCentric = !fieldCentric;
                    if (fieldCentric) {
                        imu.resetYaw();
                        holdHeadingDeg = 0;
                    }
                }
                lastY = gamepad1.y;

                // --- INTAKE (X) ---
                if (gamepad1.x && !lastX) intakeOn = !intakeOn;
                lastX = gamepad1.x;
                intakeMotor.setPower(intakeOn ? 1.0 : 0.0);

                // --- SHOOTER LEFT (LB) ---
                if (gamepad1.left_bumper && !lastLB) {
                    shooterLeftOn = !shooterLeftOn;
                    shooterLeftPid.reset();
                    shooterLeftTimer.reset();
                    shooterLeftPeakRpm = 0;
                    shooterLeftPeakCurrent = 0;
                    shooterLeftSpinUpTime = -1;
                    shooterLeftReady = false;
                    shooterLeftRumbleDone = false;
                }
                lastLB = gamepad1.left_bumper;

                // --- SHOOTER RIGHT (RB) ---
                if (gamepad1.right_bumper && !lastRB) {
                    shooterRightOn = !shooterRightOn;
                    shooterRightPid.reset();
                    shooterRightReady = false;
                    shooterRightRumbleDone = false;
                }
                lastRB = gamepad1.right_bumper;

                // --- HEADING HOLD (LS) ---
                if (gamepad1.left_stick_button && !lastLS) {
                    headingHold = !headingHold;
                    holdHeadingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                }
                lastLS = gamepad1.left_stick_button;

                // --- SERVOS (D-PAD) ---
                if (gamepad1.dpad_up && !lastUp) servo1Active = !servo1Active;
                lastUp = gamepad1.dpad_up;
                if (gamepad1.dpad_down && !lastDown) servo2Active = !servo2Active;
                lastDown = gamepad1.dpad_down;
                if (gamepad1.dpad_right && !lastRight) servo3Active = !servo3Active;
                lastRight = gamepad1.dpad_right;
                if (gamepad1.dpad_left && !lastLeft) servo4Active = !servo4Active;
                lastLeft = gamepad1.dpad_left;

                // --- SLOW MODE (B) ---
                if (gamepad1.b && !lastB) slowMode = !slowMode;
                lastB = gamepad1.b;
            }
            // Αν kill switch: όλα τα παραπάνω μεν OFF και ΜΕΝΟΥΝ OFF μέχρι re-arm.

            // --- SERVOS (kill switch κλείνει όλα στο REST) ---
            servo1.setPosition(servo1Active ? SERVO_ACTIVE : SERVO_REST);
            servo2.setPosition(servo2Active ? SERVO_ACTIVE : SERVO_REST);
            servo3.setPosition(servo3Active ? SERVO_ACTIVE : SERVO_REST);
            servo4.setPosition(servo4Active ? SERVO_ACTIVE : SERVO_REST);

            // ============================================================
            // SHOOTER VELOCITY PIDF - κλείνει στο target RPM
            // ============================================================

            double slTicksPerSec = Math.abs(shooterLeft.getVelocity());
            double slRpm = slTicksPerSec * 60.0 / SHOOTER_TICKS_PER_REV;
            double slPower = 0, slErrorRpm = 0;
            if (shooterLeftOn) {
                slPower = shooterLeftPid.calculate(shooterTargetTicksPerSec, slTicksPerSec, dt);
                shooterLeft.setPower(slPower);
            } else {
                shooterLeft.setPower(0);
            }
            slErrorRpm = SHOOTER_TARGET_RPM - slRpm;

            double srTicksPerSec = Math.abs(shooterRight.getVelocity());
            double srRpm = srTicksPerSec * 60.0 / SHOOTER_TICKS_PER_REV;
            double srPower = 0, srErrorRpm = 0;
            if (shooterRightOn) {
                srPower = shooterRightPid.calculate(shooterTargetTicksPerSec, srTicksPerSec, dt);
                shooterRight.setPower(srPower);
            } else {
                shooterRight.setPower(0);
            }
            srErrorRpm = SHOOTER_TARGET_RPM - srRpm;

            // --- READY + rumble ---
            if (shooterLeftOn && !shooterLeftReady && Math.abs(slErrorRpm) <= SHOOTER_READY_ERROR_RPM) {
                shooterLeftReady = true;
                shooterLeftSpinUpTime = shooterLeftTimer.seconds();
                if (!shooterLeftRumbleDone) {
                    gamepad1.rumbleBlips(1);
                    shooterLeftRumbleDone = true;
                }
            }
            if (!shooterLeftOn) shooterLeftReady = false;

            if (shooterRightOn && !shooterRightReady && Math.abs(srErrorRpm) <= SHOOTER_READY_ERROR_RPM) {
                shooterRightReady = true;
                if (!shooterRightRumbleDone) {
                    gamepad1.rumbleBlips(1);
                    shooterRightRumbleDone = true;
                }
            }
            if (!shooterRightOn) shooterRightReady = false;

            // --- Peak tracking (left shooter) ---
            if (shooterLeftOn) {
                if (slRpm > shooterLeftPeakRpm) shooterLeftPeakRpm = slRpm;
                if (SHOOTER_DEBUG_TELEMETRY) {
                    double slCurrent = shooterLeft.getCurrent(CurrentUnit.AMPS);
                    if (slCurrent > shooterLeftPeakCurrent) shooterLeftPeakCurrent = slCurrent;
                }
            }

            // ============================================================
            // DRIVE: deadband + expo curve + slew limit + heading hold
            // ============================================================

            double rawX = gamepad1.left_stick_x;
            double rawY = -gamepad1.left_stick_y; // μπρος = +
            double rawRx = gamepad1.right_stick_x;

            double stickX = stickCurve(rawX);
            double stickY = stickCurve(rawY);
            double stickRx = stickCurve(rawRx);

            double driveSpeed = slowMode ? SLOW_DRIVE : 1.0;
            double turnSpeed = slowMode ? SLOW_TURN : 1.0;

            double strafeCmd = stickX * driveSpeed;
            double forwardCmd = stickY * driveSpeed;
            double turnCmd = stickRx * turnSpeed;

            // --- HEADING HOLD ---
            double headingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            if (Math.abs(rawRx) < STICK_DEADBAND && headingHold) {
                double hErr = wrapDegrees(headingDeg - holdHeadingDeg);
                if (Math.abs(hErr) > HEADING_HOLD_DEADBAND_DEG) {
                    double holdPower = HEADING_HOLD_DIRECTION * HEADING_HOLD_KP * hErr;
                    turnCmd = clamp(holdPower, -HEADING_HOLD_MAX_POWER, HEADING_HOLD_MAX_POWER)
                            * turnSpeed;
                } else {
                    turnCmd = 0;
                }
            } else {
                holdHeadingDeg = headingDeg; // ο driver ενεργός -> νέο reference
            }

            // --- Slew rate limit ---
            double driveStep = DRIVE_SLEW_RATE_PER_SEC * dt;
            double turnStep = TURN_SLEW_RATE_PER_SEC * dt;

            strafeCmd = slew(strafeCmd, lastStrafeCmd, driveStep);
            forwardCmd = slew(forwardCmd, lastForwardCmd, driveStep);
            turnCmd = slew(turnCmd, lastTurnCmd, turnStep);

            lastStrafeCmd = strafeCmd;
            lastForwardCmd = forwardCmd;
            lastTurnCmd = turnCmd;

            // --- ΚΙΝΗΣΗ ---
            if (fieldCentric) {
                drive.driveFieldCentric(strafeCmd, forwardCmd, turnCmd, headingDeg);
            } else {
                drive.driveRobotCentric(strafeCmd, forwardCmd, turnCmd);
            }

            // ============================================================
            // TELEMETRY (throttled: κάθε 3 loops -> υψηλότερο control rate)
            // ============================================================

            if (++telemetryCounter >= 3) {
                telemetryCounter = 0;

                double intakeRpm = Math.abs(intakeMotor.getVelocity())
                        * 60.0 / INTAKE_TICKS_PER_REV;

                telemetry.addData("Drive Mode", fieldCentric ? "FIELD CENTRIC" : "ROBOT CENTRIC");
                telemetry.addData("Slow Mode (B)", slowMode ? "ON - 30%" : "OFF");
                telemetry.addData("Heading", "%.1f° (hold %.0f°) %s",
                        headingDeg, holdHeadingDeg, headingHold ? "HOLD ON" : "HOLD OFF");
                telemetry.addData("KILL SWITCH", killSwitch ? "*** ENGAGED ***" : "armed");

                telemetry.addData("Intake (X)", intakeOn ? "ON" : "OFF");
                telemetry.addData("Intake RPM", "%.0f", intakeRpm);

                telemetry.addData("Shooter L (LB)", shooterLeftOn ? "ON" : "OFF");
                telemetry.addData("Shooter L", "cur=%6.0f err=%+5.0f %s",
                        slRpm, slErrorRpm, shooterLeftReady ? "READY" : "spin-up");
                telemetry.addData("Shooter L PID", "pwr=%.2f I=%.1f",
                        slPower, shooterLeftPid.getIntegral());
                telemetry.addData("Shooter L ticks", shooterLeft.getCurrentPosition());

                if (SHOOTER_DEBUG_TELEMETRY) {
                    telemetry.addData("Shooter L peak RPM", "%.0f", shooterLeftPeakRpm);
                    telemetry.addData("Shooter L peak A", "%.2f", shooterLeftPeakCurrent);
                    telemetry.addData("Shooter L spin-up (s)", "%.2f", shooterLeftSpinUpTime);
                }

                telemetry.addData("Shooter R (RB)", shooterRightOn ? "ON" : "OFF");
                telemetry.addData("Shooter R", "cur=%6.0f err=%+5.0f %s",
                        srRpm, srErrorRpm, shooterRightReady ? "READY" : "spin-up");
                telemetry.addData("Shooter R PID", "pwr=%.2f I=%.1f",
                        srPower, shooterRightPid.getIntegral());

                telemetry.addData("Servo1 (UP)", "%s  pos=%.2f",
                        servo1Active ? "ACTIVE" : "REST", servo1.getPosition());
                telemetry.addData("Servo2 (DOWN)", "%s  pos=%.2f",
                        servo2Active ? "ACTIVE" : "REST", servo2.getPosition());
                telemetry.addData("Servo3 (RIGHT)", "%s  pos=%.2f",
                        servo3Active ? "ACTIVE" : "REST", servo3.getPosition());
                telemetry.addData("Servo4 (LEFT)", "%s  pos=%.2f",
                        servo4Active ? "ACTIVE" : "REST", servo4.getPosition());

                telemetry.update();
            }
        }

        // Σταμάτα τα motors όταν τελειώσει το OpMode
        drive.stop();
        intakeMotor.setPower(0);
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }

    // =========================================================================
    // HELPERS
    // =========================================================================

    /** Deadband + exponential stick curve (κρατάει το πρόσημο). */
    private static double stickCurve(double input) {
        if (Math.abs(input) < STICK_DEADBAND) return 0;
        double magnitude = Math.pow(
                (Math.abs(input) - STICK_DEADBAND) / (1.0 - STICK_DEADBAND), STICK_EXPONENT);
        double shaped = STICK_DEADBAND + magnitude * (1.0 - STICK_DEADBAND);
        return Math.signum(input) * shaped;
    }

    /** Slew-rate step: μέγιστη αλλαγή maxStep από lastOut προς target. */
    private static double slew(double target, double lastOut, double maxStep) {
        double delta = clamp(target - lastOut, -maxStep, maxStep);
        return lastOut + delta;
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    /** Wrap (-180, +180]. */
    private static double wrapDegrees(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }
}
