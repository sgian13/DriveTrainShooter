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
import com.qualcomm.robotcore.hardware.VoltageSensor;
/*
 * ΚΟΥΜΠΙΑ (gamepad1):
 *   LB/RB shooters ON/OFF -> κλείνουν σε SHOOTER_TARGET_RPM (PIDF + rumble στο READY)
 *   X intake, B slow mode, Y field/robot-centric, LS heading hold,
 *   START reset yaw / re-arm after kill switch, BACK kill switch
 *   D-PAD: UP/DOWN/RIGHT/LEFT = servo1-4
 */
@TeleOp(name = "Grobo-DriveTrain_Shooter_Servo")
public class DriveTrainServoShooter2 extends LinearOpMode {

    // ---- shooter (kF υπολογίζεται αυτόματα, kP/kI/kD tune εδώ) ----
    private static final double SHOOTER_TICKS_PER_REV = 28;
    private static final double SHOOTER_MAX_RPM = 6000;
    private static final double SHOOTER_TARGET_RPM = 4000;
    private static final double SHOOTER_KP = 0.00050;   // sim-validated (500 trials)
    private static final double SHOOTER_KI = 0.00060;
    private static final double SHOOTER_KD = 0.00002;
    private static final double SHOOTER_INTEGRAL_LIMIT = 1500;
    private static final double SHOOTER_INTEGRAL_DEADZONE = 40;
    private static final double SHOOTER_READY_ERROR_RPM = 120;
    private static final boolean SHOOTER_DEBUG_TELEMETRY = false;

    // ---- αντιστάθμιση μπαταρίας: ισχύς * 12.5/Vbus, clamped ----
    private static final boolean ENABLE_VOLTAGE_COMPENSATION = true;
    private static final double NOMINAL_VOLTAGE = 12.5;

    private static final double INTAKE_TICKS_PER_REV = 28;

    // ---- drive ----
    private static final double STICK_DEADBAND = 0.05;
    private static final double STICK_EXPONENT = 1.6;
    private static final double SLOW_DRIVE = 0.30;
    private static final double SLOW_TURN = 0.30;
    private static final double DRIVE_SLEW_PER_SEC = 6.0;
    private static final double TURN_SLEW_PER_SEC = 8.0;

    // ---- heading hold (LS) — ανάποδα; βάλε DIRECTION = -1 ----
    private static final boolean HEADING_HOLD_DEFAULT = true;
    private static final double HEADING_HOLD_KP = 0.010;
    private static final double HEADING_HOLD_MAX_POWER = 0.45;
    private static final double HEADING_HOLD_DEADBAND_DEG = 1.0;
    private static final double HEADING_HOLD_DIRECTION = 1.0;

    private static final double SERVO_REST = 0.1;
    private static final double SERVO_ACTIVE = 0.9;

    private double shooterTargetTicks, kFeedForward;
    private double holdHeadingDeg = 0;
    private double lastLoopTime = 0;
    private int telemetryTicker = 0;

    @Override
    public void runOpMode() {

        // ---------- hardware ----------
        Motor frontLeft = new Motor(hardwareMap, "leftFMotor");
        Motor frontRight = new Motor(hardwareMap, "rightFMotor");
        Motor backLeft = new Motor(hardwareMap, "leftBMotor");
        Motor backRight = new Motor(hardwareMap, "rightBMotor");

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);

        DcMotorEx shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        DcMotorEx shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
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
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
        imu.resetYaw();

        // ---------- PIDF ----------
        shooterTargetTicks = SHOOTER_TARGET_RPM / 60.0 * SHOOTER_TICKS_PER_REV;
        kFeedForward = 1.0 / (SHOOTER_MAX_RPM / 60.0 * SHOOTER_TICKS_PER_REV);
        GroboControl.VelocityPIDF pidL = new GroboControl.VelocityPIDF(
                SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, kFeedForward, SHOOTER_INTEGRAL_LIMIT,
                SHOOTER_INTEGRAL_DEADZONE);
        GroboControl.VelocityPIDF pidR = new GroboControl.VelocityPIDF(
                SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, kFeedForward, SHOOTER_INTEGRAL_LIMIT,
                SHOOTER_INTEGRAL_DEADZONE);

        ElapsedTime shooterTimer = new ElapsedTime();
        ElapsedTime loopTimer = new ElapsedTime();
        double slPeakRpm = 0, slPeakAmps = 0, slSpinUp = -1;
        boolean slReady = false, slRumbled = false;
        boolean srReady = false, srRumbled = false;

        // ---------- state ----------
        boolean fieldCentric = false, lastY = false, lastStart = false;
        boolean intakeOn = false, lastX = false;
        boolean shooterL = false, lastLB = false;
        boolean shooterR = false, lastRB = false;
        boolean slowMode = false, lastB = false;
        boolean headingHold = HEADING_HOLD_DEFAULT, lastLS = false;
        boolean killSwitch = false, lastBack = false;
        boolean[] servoActive = new boolean[4];
        boolean[] lastPad = new boolean[4];
        double lastStrafe = 0, lastForward = 0, lastTurn = 0;

        telemetry.addData("Status", "Ready (PIDF)");
        telemetry.addData("Target", "%.0f RPM (kF=%.6f)", SHOOTER_TARGET_RPM, kFeedForward);
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {

            double now = loopTimer.seconds();
            double dt = now - lastLoopTime;
            lastLoopTime = now;
            if (!(dt > 0) || dt > 0.5) dt = 0.02;

            // ---------- kill switch / re-arm ----------
            if (gamepad1.back && !lastBack) {
                killSwitch = true;
                intakeOn = shooterL = shooterR = false;
                for (int i = 0; i < 4; i++) servoActive[i] = false;
                pidL.reset();
                pidR.reset();
            }
            lastBack = gamepad1.back;

            if (gamepad1.start && !lastStart) {
                imu.resetYaw();
                holdHeadingDeg = 0;
                killSwitch = false;
            }
            lastStart = gamepad1.start;

            if (!killSwitch) {
                if (gamepad1.y && !lastY) {
                    fieldCentric = !fieldCentric;
                    if (fieldCentric) {
                        imu.resetYaw();
                        holdHeadingDeg = 0;
                    }
                }
                lastY = gamepad1.y;

                if (gamepad1.x && !lastX) intakeOn = !intakeOn;
                lastX = gamepad1.x;
                intake.setPower(intakeOn ? 1.0 : 0.0);

                if (gamepad1.left_bumper && !lastLB) {
                    shooterL = !shooterL;
                    pidL.reset();
                    shooterTimer.reset();
                    slPeakRpm = 0;
                    slSpinUp = -1;
                    slReady = slRumbled = false;
                }
                lastLB = gamepad1.left_bumper;

                if (gamepad1.right_bumper && !lastRB) {
                    shooterR = !shooterR;
                    pidR.reset();
                    srReady = srRumbled = false;
                }
                lastRB = gamepad1.right_bumper;

                if (gamepad1.left_stick_button && !lastLS) {
                    headingHold = !headingHold;
                    holdHeadingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                }
                lastLS = gamepad1.left_stick_button;

                boolean[] pad = {gamepad1.dpad_up, gamepad1.dpad_down,
                        gamepad1.dpad_right, gamepad1.dpad_left};
                for (int i = 0; i < 4; i++) {
                    if (pad[i] && !lastPad[i]) servoActive[i] = !servoActive[i];
                    lastPad[i] = pad[i];
                }

                if (gamepad1.b && !lastB) slowMode = !slowMode;
                lastB = gamepad1.b;
            }

            Servo[] servos = {servo1, servo2, servo3, servo4};
            for (int i = 0; i < 4; i++)
                servos[i].setPosition(servoActive[i] ? SERVO_ACTIVE : SERVO_REST);

            // ---------- shooters (PIDF -> target RPM) ----------
            double vBus = ENABLE_VOLTAGE_COMPENSATION ? hubVoltage() : NOMINAL_VOLTAGE;
            double comp = GroboControl.clamp(NOMINAL_VOLTAGE / vBus, 0.7, 1.2);

            double slTps = Math.abs(shooterLeft.getVelocity());
            double slRpm = slTps * 60.0 / SHOOTER_TICKS_PER_REV;
            double slPower = 0, slError = 0;
            if (shooterL) {
                slPower = GroboControl.clamp(
                        pidL.calculate(shooterTargetTicks, slTps, dt) * comp, 0.0, 1.0);
                shooterLeft.setPower(slPower);
            } else {
                shooterLeft.setPower(0);
            }
            slError = SHOOTER_TARGET_RPM - slRpm;

            double srTps = Math.abs(shooterRight.getVelocity());
            double srRpm = srTps * 60.0 / SHOOTER_TICKS_PER_REV;
            double srPower = 0, srError = 0;
            if (shooterR) {
                srPower = GroboControl.clamp(
                        pidR.calculate(shooterTargetTicks, srTps, dt) * comp, 0.0, 1.0);
                shooterRight.setPower(srPower);
            } else {
                shooterRight.setPower(0);
            }
            srError = SHOOTER_TARGET_RPM - srRpm;

            if (shooterL && !slReady && Math.abs(slError) <= SHOOTER_READY_ERROR_RPM) {
                slReady = true;
                slSpinUp = shooterTimer.seconds();
                if (!slRumbled) {
                    gamepad1.rumbleBlips(1);
                    slRumbled = true;
                }
            }
            if (shooterR && !srReady && Math.abs(srError) <= SHOOTER_READY_ERROR_RPM) {
                srReady = true;
                if (!srRumbled) {
                    gamepad1.rumbleBlips(1);
                    srRumbled = true;
                }
            }

            if (shooterL) {
                if (slRpm > slPeakRpm) slPeakRpm = slRpm;
                if (SHOOTER_DEBUG_TELEMETRY) {
                    double amps = shooterLeft.getCurrent(CurrentUnit.AMPS);
                    if (amps > slPeakAmps) slPeakAmps = amps;
                }
            }

            // ---------- drive ----------
            double driveSpeed = slowMode ? SLOW_DRIVE : 1.0;
            double turnSpeed = slowMode ? SLOW_TURN : 1.0;

            double strafe = GroboControl.stickCurve(
                    gamepad1.left_stick_x, STICK_DEADBAND, STICK_EXPONENT) * driveSpeed;
            double forward = GroboControl.stickCurve(
                    -gamepad1.left_stick_y, STICK_DEADBAND, STICK_EXPONENT) * driveSpeed;

            double headingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double rawRx = gamepad1.right_stick_x;
            double turn;
            if (Math.abs(rawRx) < STICK_DEADBAND && headingHold) {
                double hErr = GroboControl.wrapDegrees(headingDeg - holdHeadingDeg);
                turn = Math.abs(hErr) > HEADING_HOLD_DEADBAND_DEG
                        ? GroboControl.clamp(
                        HEADING_HOLD_DIRECTION * HEADING_HOLD_KP * hErr,
                        -HEADING_HOLD_MAX_POWER, HEADING_HOLD_MAX_POWER) * turnSpeed
                        : 0;
            } else {
                holdHeadingDeg = headingDeg;
                turn = GroboControl.stickCurve(rawRx, STICK_DEADBAND, STICK_EXPONENT) * turnSpeed;
            }

            strafe = GroboControl.slew(strafe, lastStrafe, DRIVE_SLEW_PER_SEC, dt);
            forward = GroboControl.slew(forward, lastForward, DRIVE_SLEW_PER_SEC, dt);
            turn = GroboControl.slew(turn, lastTurn, TURN_SLEW_PER_SEC, dt);
            lastStrafe = strafe;
            lastForward = forward;
            lastTurn = turn;

            if (fieldCentric) drive.driveFieldCentric(strafe, forward, turn, headingDeg);
            else drive.driveRobotCentric(strafe, forward, turn);

            // ---------- telemetry (update κάθε 3 loops) ----------
            if (++telemetryTicker >= 3) {
                telemetryTicker = 0;
                double intakeRpm = Math.abs(intake.getVelocity()) * 60.0 / INTAKE_TICKS_PER_REV;

                telemetry.addData("Mode", "%s | slow=%s | kill=%s",
                        fieldCentric ? "FIELD" : "ROBOT", slowMode, killSwitch);
                telemetry.addData("Heading", "%.1f° (hold %.0f°)", headingDeg, holdHeadingDeg);
                telemetry.addData("Vbus", "%.2f", vBus);
                telemetry.addData("Intake", "%s %.0f RPM", intakeOn ? "ON" : "OFF", intakeRpm);
                telemetry.addData("Shooter L", "%s cur=%.0f err=%+.0f %s",
                        shooterL ? "ON" : "OFF", slRpm, slError, slReady ? "READY" : "spin-up");
                telemetry.addData("Shooter L PID", "pwr=%.2f I=%.1f", slPower, pidL.getIntegral());
                telemetry.addData("Shooter R", "%s cur=%.0f err=%+.0f %s",
                        shooterR ? "ON" : "OFF", srRpm, srError, srReady ? "READY" : "spin-up");
                telemetry.addData("Shooter R PID", "pwr=%.2f I=%.1f", srPower, pidR.getIntegral());

                if (SHOOTER_DEBUG_TELEMETRY) {
                    telemetry.addData("Shooter L peak RPM", "%.0f", slPeakRpm);
                    telemetry.addData("Shooter L peak A", "%.2f", slPeakAmps);
                    telemetry.addData("Shooter L spin-up", "%.2f s", slSpinUp);
                }

                String padNames = "UDRL";
                for (int i = 0; i < 4; i++)
                    telemetry.addData("Servo" + (i + 1) + " (" + padNames.charAt(i) + ")",
                            "%s pos=%.2f", servoActive[i] ? "ACTIVE" : "REST",
                            servos[i].getPosition());

                telemetry.update();
            }
        }

        drive.stop();
        intake.setPower(0);
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }

    /** Τάση μπαταρίας: ψάχνει όλους τους voltage sensors και κρατά την πιο χειρότερη. */
    private double hubVoltage() {
        double worst = NOMINAL_VOLTAGE;
        try {
            for (VoltageSensor s : hardwareMap.getAll(VoltageSensor.class)) {
                double v = s.getVoltage();
                if (v > 6.0 && !Double.isNaN(v) && v < worst) worst = v;
            }
        } catch (Exception ignored) {
            // no voltage sensor readable -> keep nominal
        }
        return worst;
    }
}
