package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.qualcomm.hardware.lynx.LynxModule;
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
 * ΚΟΥΜΠΙΑ (gamepad1):
 * Αριστερό stick -> κίνηση
 * Δεξί stick X   -> στροφή
 * START          -> re-arm (αν kill switch) / reset yaw (αν οπλισμένο)
 * LB / RB        -> shooter left / right ON/OFF
 * LS             -> heading hold ON/OFF
 * B              -> slow mode
 * Y              -> field-centric / robot-centric
 * X              -> intake
 * BACK           -> kill switch
 * D-PAD          -> servos (UP=1, DOWN=2, RIGHT=3, LEFT=4)
 */

@TeleOp(name = "Grobo-DriveTrain_Shooter_ServoMAIN")
public class DriveTrainServoShooterNEW extends LinearOpMode {

    private static final double SHOOTER_TICKS_PER_REV = 28;
    private static final double SHOOTER_MAX_RPM = 6000;

    private static final double SHOOTER_LEFT_TARGET_RPM = 6000;
    private static final double SHOOTER_RIGHT_TARGET_RPM = 6000;

    private static final double SHOOTER_KP = 0.00030;
    private static final double SHOOTER_KI = 0.00003;
    private static final double SHOOTER_KD = 0.00001;

    private static final double SHOOTER_INTEGRAL_LIMIT = 6000;
    private static final double SHOOTER_READY_ERROR_RPM = 120.0;

    private static final boolean SHOOTER_DEBUG_TELEMETRY = false;

    private static final double INTAKE_TICKS_PER_REV = 28;

    private static final double STICK_DEADBAND = 0.05;
    private static final double STICK_EXPONENT = 1.6;

    private static final double SLOW_DRIVE = 0.30;
    private static final double SLOW_TURN = 0.30;

    private static final boolean HEADING_HOLD_DEFAULT = true;
    private static final double HEADING_HOLD_KP = 0.010;
    private static final double HEADING_HOLD_MAX_POWER = 0.45;
    private static final double HEADING_HOLD_DEADBAND_DEG = 1.0;
    private static final double HEADING_HOLD_DIRECTION = 1.0;
    private static final double HEADING_HOLD_SETTLE_SEC = 0.25;

    private static final double DRIVE_SLEW_RATE_PER_SEC = 6.0;
    private static final double TURN_SLEW_RATE_PER_SEC = 8.0;

    private static final double SERVO_REST = 0.1;
    private static final double SERVO_ACTIVE = 0.9;

    private static class Edge {
        private boolean last = false;

        boolean pressed(boolean now) {
            boolean result = now && !last;
            last = now;
            return result;
        }
    }

    private static class GroboControl {

        private final double kP, kI, kD, kF;
        private final double integralLimit;

        private double integral = 0;
        private double lastError = 0;
        private double smoothedDerivative = 0;
        private boolean firstRun = true;

        GroboControl(double kP, double kI, double kD, double kF, double integralLimit) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            this.integralLimit = integralLimit;
        }

        void reset() {
            integral = 0;
            lastError = 0;
            smoothedDerivative = 0;
            firstRun = true;
        }

        double calculate(double targetTicksPerSec, double currentTicksPerSec, double dtVal) {
            double safeDt = (!(dtVal > 0) || dtVal > 0.5) ? 0.02 : dtVal;

            double error = targetTicksPerSec - currentTicksPerSec;

            double derivative;
            if (!firstRun) {
                double raw = (error - lastError) / safeDt;
                smoothedDerivative = 0.5 * raw + 0.5 * smoothedDerivative;
                derivative = smoothedDerivative;
            } else {
                firstRun = false;
                derivative = 0;
            }
            lastError = error;

            double feedForward = kF * targetTicksPerSec;
            double pTerm = kP * error;
            double dTerm = kD * derivative;

            double candidateIntegral = clamp(
                    integral + error * safeDt,
                    -integralLimit,
                    integralLimit
            );

            double unsaturated = feedForward + pTerm + kI * candidateIntegral + dTerm;

            boolean saturatedHigh = unsaturated > 1.0 && error > 0;
            boolean saturatedLow = unsaturated < 0.0 && error < 0;

            if (!saturatedHigh && !saturatedLow) {
                integral = candidateIntegral;
            }

            double output = feedForward + pTerm + kI * integral + dTerm;

            return clamp(output, 0.0, 1.0);
        }

        double getIntegral() {
            return integral;
        }
    }

    private static class Shooter {

        final DcMotorEx motor;
        final GroboControl pid;
        final double targetRpm;
        final double targetTicksPerSec;
        final ElapsedTime timer = new ElapsedTime();

        boolean on = false;
        boolean ready = false;
        boolean rumbleDone = false;

        double rpm = 0;
        double errorRpm = 0;
        double power = 0;

        double peakRpm = 0;
        double peakCurrent = 0;
        double spinUpTime = -1;

        Shooter(DcMotorEx motor, double targetRpm, double kF, DcMotorSimple.Direction direction) {
            this.motor = motor;
            this.targetRpm = targetRpm;
            this.targetTicksPerSec = targetRpm / 60.0 * SHOOTER_TICKS_PER_REV;
            this.pid = new GroboControl(
                    SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, kF, SHOOTER_INTEGRAL_LIMIT
            );

            motor.setDirection(direction);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setPower(0);
        }

        private void resetState() {
            pid.reset();
            timer.reset();
            ready = false;
            rumbleDone = false;
            peakRpm = 0;
            peakCurrent = 0;
            spinUpTime = -1;
        }

        void toggle() {
            on = !on;
            resetState();
        }

        void stop() {
            on = false;
            resetState();
            motor.setPower(0);
        }

        boolean update(boolean killSwitch, double dt) {

            double ticksPerSec = Math.abs(motor.getVelocity());
            rpm = ticksPerSec * 60.0 / SHOOTER_TICKS_PER_REV;
            errorRpm = targetRpm - rpm;

            boolean justReady = false;

            if (!killSwitch && on) {

                power = pid.calculate(targetTicksPerSec, ticksPerSec, dt);
                motor.setPower(power);

                ready = Math.abs(errorRpm) <= SHOOTER_READY_ERROR_RPM;

                if (ready && !rumbleDone) {
                    rumbleDone = true;
                    spinUpTime = timer.seconds();
                    justReady = true;
                }

                if (rpm > peakRpm) {
                    peakRpm = rpm;
                }

                if (SHOOTER_DEBUG_TELEMETRY) {
                    double current = motor.getCurrent(CurrentUnit.AMPS);
                    if (current > peakCurrent) {
                        peakCurrent = current;
                    }
                }

            } else {
                power = 0;
                motor.setPower(0);
                pid.reset();
                ready = false;
            }

            return justReady;
        }
    }

    @Override
    public void runOpMode() {

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        boolean fieldCentric = false;
        boolean intakeOn = false;
        boolean slowMode = false;
        boolean headingHold = HEADING_HOLD_DEFAULT;
        boolean killSwitch = false;

        boolean[] servoActive = new boolean[4];

        Edge backEdge = new Edge();
        Edge startEdge = new Edge();
        Edge yEdge = new Edge();
        Edge xEdge = new Edge();
        Edge bEdge = new Edge();
        Edge lbEdge = new Edge();
        Edge rbEdge = new Edge();
        Edge lsEdge = new Edge();
        Edge[] dpadEdges = {new Edge(), new Edge(), new Edge(), new Edge()};

        ElapsedTime loopTimer = new ElapsedTime();
        double lastLoopTime = 0;

        ElapsedTime headingSettleTimer = new ElapsedTime();

        int telemetryCounter = 0;

        double lastStrafeCmd = 0;
        double lastForwardCmd = 0;
        double lastTurnCmd = 0;

        double holdHeadingDeg = 0;

        // Hardware initialization with safety try-catch
        Motor frontLeft = null;
        Motor frontRight = null;
        Motor backLeft = null;
        Motor backRight = null;
        DcMotorEx intakeMotor = null;
        Shooter shooterLeft = null;
        Shooter shooterRight = null;
        IMU imu = null;

        boolean hardwareInitSuccess = true;
        boolean driveOk = true;
        StringBuilder initErrorMsg = new StringBuilder();

        try {
            frontLeft = new Motor(hardwareMap, "leftFMotor");
            frontRight = new Motor(hardwareMap, "rightFMotor");
            backLeft = new Motor(hardwareMap, "leftBMotor");
            backRight = new Motor(hardwareMap, "rightBMotor");
        } catch (Exception e) {
            driveOk = false;
            hardwareInitSuccess = false;
            initErrorMsg.append("Drive Motors missing. ");
        }

        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setPower(0);
        } catch (Exception e) {
            intakeMotor = null;
            hardwareInitSuccess = false;
            initErrorMsg.append("intakeMotor missing. ");
        }

        double shooterMaxTicksPerSec =
                SHOOTER_MAX_RPM / 60.0 * SHOOTER_TICKS_PER_REV;
        double kFeedForward = 1.0 / shooterMaxTicksPerSec;

        try {
            shooterLeft = new Shooter(
                    hardwareMap.get(DcMotorEx.class, "shooterLeft"),
                    SHOOTER_LEFT_TARGET_RPM,
                    kFeedForward,
                    DcMotorSimple.Direction.REVERSE
            );
        } catch (Exception e) {
            hardwareInitSuccess = false;
            initErrorMsg.append("shooterLeft missing. ");
        }

        try {
            shooterRight = new Shooter(
                    hardwareMap.get(DcMotorEx.class, "shooterRight"),
                    SHOOTER_RIGHT_TARGET_RPM,
                    kFeedForward,
                    DcMotorSimple.Direction.FORWARD
            );
        } catch (Exception e) {
            hardwareInitSuccess = false;
            initErrorMsg.append("shooterRight missing. ");
        }

        String[] servoNames = {"servo1", "servo2", "servo3", "servo4"};
        Servo[] servos = new Servo[4];

        for (int i = 0; i < servos.length; i++) {
            try {
                servos[i] = hardwareMap.get(Servo.class, servoNames[i]);
                servos[i].setDirection(Servo.Direction.FORWARD);
                servos[i].setPosition(SERVO_REST);
            } catch (Exception e) {
                servos[i] = null;
                hardwareInitSuccess = false;
                initErrorMsg.append(servoNames[i]).append(" missing. ");
            }
        }

        MecanumDrive drive = null;
        if (driveOk) {
            drive = new MecanumDrive(
                    frontLeft,
                    frontRight,
                    backLeft,
                    backRight
            );
        }

        try {
            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot orientation =
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                    );

            imu.initialize(new IMU.Parameters(orientation));
            imu.resetYaw();
        } catch (Exception e) {
            imu = null;
            hardwareInitSuccess = false;
            initErrorMsg.append("IMU missing. ");
        }

        if (!hardwareInitSuccess) {
            telemetry.addData("INIT ERROR", initErrorMsg.toString());
        } else {
            telemetry.addData("Status", "Ready (PIDF edition)");
            telemetry.addData("Drive Mode", "ROBOT CENTRIC");
            telemetry.addData("Shooter L Target", "%.0f RPM", SHOOTER_LEFT_TARGET_RPM);
            telemetry.addData("Shooter R Target", "%.0f RPM", SHOOTER_RIGHT_TARGET_RPM);
        }

        telemetry.update();

        waitForStart();

        loopTimer.reset();
        headingSettleTimer.reset();

        while (opModeIsActive()) {

            double nowSec = loopTimer.seconds();
            double dt = nowSec - lastLoopTime;
            lastLoopTime = nowSec;

            if (!(dt > 0) || dt > 0.5) {
                dt = 0.02;
            }

            double loopHz = 1.0 / dt;

            boolean backPressed = backEdge.pressed(gamepad1.back);
            boolean startPressed = startEdge.pressed(gamepad1.start);
            boolean yPressed = yEdge.pressed(gamepad1.y);
            boolean xPressed = xEdge.pressed(gamepad1.x);
            boolean bPressed = bEdge.pressed(gamepad1.b);
            boolean lbPressed = lbEdge.pressed(gamepad1.left_bumper);
            boolean rbPressed = rbEdge.pressed(gamepad1.right_bumper);
            boolean lsPressed = lsEdge.pressed(gamepad1.left_stick_button);

            boolean[] dpadPressed = new boolean[4];
            dpadPressed[0] = dpadEdges[0].pressed(gamepad1.dpad_up);
            dpadPressed[1] = dpadEdges[1].pressed(gamepad1.dpad_down);
            dpadPressed[2] = dpadEdges[2].pressed(gamepad1.dpad_right);
            dpadPressed[3] = dpadEdges[3].pressed(gamepad1.dpad_left);

            double headingDeg = (imu != null)
                    ? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
                    : 0.0;

            if (backPressed) {

                killSwitch = true;

                intakeOn = false;
                if (shooterLeft != null) shooterLeft.stop();
                if (shooterRight != null) shooterRight.stop();

                for (int i = 0; i < servoActive.length; i++) {
                    servoActive[i] = false;
                }
            }

            if (startPressed) {
                if (killSwitch) {
                    killSwitch = false;
                    holdHeadingDeg = headingDeg;
                    headingSettleTimer.reset();
                } else {
                    if (imu != null) imu.resetYaw();
                    holdHeadingDeg = 0;
                    headingSettleTimer.reset();
                    headingDeg = 0;
                }
            }

            if (!killSwitch) {

                if (yPressed) {
                    fieldCentric = !fieldCentric;
                }

                if (xPressed) {
                    intakeOn = !intakeOn;
                }

                if (lbPressed && shooterLeft != null) {
                    shooterLeft.toggle();
                }

                if (rbPressed && shooterRight != null) {
                    shooterRight.toggle();
                }

                if (lsPressed) {
                    headingHold = !headingHold;
                    holdHeadingDeg = headingDeg;
                    headingSettleTimer.reset();
                }

                for (int i = 0; i < servoActive.length; i++) {
                    if (dpadPressed[i]) {
                        servoActive[i] = !servoActive[i];
                    }
                }

                if (bPressed) {
                    slowMode = !slowMode;
                }
            }

            if (intakeMotor != null) {
                intakeMotor.setPower((!killSwitch && intakeOn) ? 1.0 : 0.0);
            }

            for (int i = 0; i < servos.length; i++) {
                if (servos[i] != null) {
                    servos[i].setPosition(
                            (!killSwitch && servoActive[i]) ? SERVO_ACTIVE : SERVO_REST
                    );
                }
            }

            if (shooterLeft != null && shooterLeft.update(killSwitch, dt)) {
                gamepad1.rumbleBlips(1);
            }

            if (shooterRight != null && shooterRight.update(killSwitch, dt)) {
                gamepad1.rumbleBlips(1);
            }

            double rawX = gamepad1.left_stick_x;
            double rawY = -gamepad1.left_stick_y;
            double rawRx = gamepad1.right_stick_x;

            double stickX = stickCurve(rawX);
            double stickY = stickCurve(rawY);
            double stickRx = stickCurve(rawRx);

            double driveSpeed = slowMode ? SLOW_DRIVE : 1.0;
            double turnSpeed = slowMode ? SLOW_TURN : 1.0;

            double strafeCmd = stickX * driveSpeed;
            double forwardCmd = stickY * driveSpeed;
            double turnCmd = stickRx * turnSpeed;

            boolean driverTurning = Math.abs(rawRx) >= STICK_DEADBAND;

            if (killSwitch || !headingHold || driverTurning) {

                holdHeadingDeg = headingDeg;
                headingSettleTimer.reset();

            } else if (headingSettleTimer.seconds() < HEADING_HOLD_SETTLE_SEC) {

                holdHeadingDeg = headingDeg;
                turnCmd = 0;

            } else {

                double hErr = wrapDegrees(headingDeg - holdHeadingDeg);

                if (Math.abs(hErr) > HEADING_HOLD_DEADBAND_DEG) {

                    double holdPower =
                            HEADING_HOLD_DIRECTION * HEADING_HOLD_KP * hErr;

                    turnCmd = clamp(
                            holdPower,
                            -HEADING_HOLD_MAX_POWER,
                            HEADING_HOLD_MAX_POWER
                    ) * turnSpeed;

                } else {
                    turnCmd = 0;
                }
            }

            if (killSwitch) {

                lastStrafeCmd = 0;
                lastForwardCmd = 0;
                lastTurnCmd = 0;

                if (drive != null) {
                    drive.stop();
                }

            } else {

                double driveStep = DRIVE_SLEW_RATE_PER_SEC * dt;
                double turnStep = TURN_SLEW_RATE_PER_SEC * dt;

                strafeCmd = slew(strafeCmd, lastStrafeCmd, driveStep);
                forwardCmd = slew(forwardCmd, lastForwardCmd, driveStep);
                turnCmd = slew(turnCmd, lastTurnCmd, turnStep);

                lastStrafeCmd = strafeCmd;
                lastForwardCmd = forwardCmd;
                lastTurnCmd = turnCmd;

                if (drive != null) {
                    if (fieldCentric) {
                        drive.driveFieldCentric(strafeCmd, forwardCmd, turnCmd, headingDeg);
                    } else {
                        drive.driveRobotCentric(strafeCmd, forwardCmd, turnCmd);
                    }
                }
            }

            if (++telemetryCounter >= 3) {

                telemetryCounter = 0;

                double intakeRpm = (intakeMotor != null)
                        ? Math.abs(intakeMotor.getVelocity()) * 60.0 / INTAKE_TICKS_PER_REV
                        : 0.0;

                if (!hardwareInitSuccess) {
                    telemetry.addData("INIT ERROR", initErrorMsg.toString());
                }

                telemetry.addData("Loop Rate", "%.1f Hz", loopHz);

                telemetry.addData(
                        "Drive Mode",
                        fieldCentric ? "FIELD CENTRIC" : "ROBOT CENTRIC"
                );

                telemetry.addData("Slow Mode (B)", slowMode ? "ON - 30%" : "OFF");

                telemetry.addData(
                        "Heading",
                        "%.1f° (hold %.0f°) %s",
                        headingDeg,
                        holdHeadingDeg,
                        headingHold ? "HOLD ON" : "HOLD OFF"
                );

                telemetry.addData(
                        "KILL SWITCH",
                        killSwitch ? "*** ENGAGED ***" : "armed"
                );

                telemetry.addData("Intake (X)", intakeOn ? "ON" : "OFF");
                telemetry.addData("Intake RPM", "%.0f", intakeRpm);

                if (shooterLeft != null) {
                    addShooterTelemetry("L (LB)", shooterLeft);
                    telemetry.addData("Shooter L ticks", shooterLeft.motor.getCurrentPosition());

                    if (SHOOTER_DEBUG_TELEMETRY) {
                        telemetry.addData("Shooter L peak RPM", "%.0f", shooterLeft.peakRpm);
                        telemetry.addData("Shooter L peak A", "%.2f", shooterLeft.peakCurrent);
                        telemetry.addData("Shooter L spin-up (s)", "%.2f", shooterLeft.spinUpTime);
                    }
                }

                if (shooterRight != null) {
                    addShooterTelemetry("R (RB)", shooterRight);
                }

                String[] servoLabels = {"UP", "DOWN", "RIGHT", "LEFT"};

                for (int i = 0; i < servos.length; i++) {
                    if (servos[i] != null) {
                        telemetry.addData(
                                "Servo" + (i + 1) + " (" + servoLabels[i] + ")",
                                "%s pos=%.2f",
                                servoActive[i] ? "ACTIVE" : "REST",
                                servos[i].getPosition()
                        );
                    }
                }

                telemetry.update();
            }
        }

        if (drive != null) {
            drive.stop();
        }

        if (intakeMotor != null) intakeMotor.setPower(0);
        if (shooterLeft != null) shooterLeft.stop();
        if (shooterRight != null) shooterRight.stop();

        for (Servo servo : servos) {
            if (servo != null) {
                servo.setPosition(SERVO_REST);
            }
        }
    }

    private void addShooterTelemetry(String label, Shooter s) {

        telemetry.addData("Shooter " + label, s.on ? "ON" : "OFF");

        telemetry.addData(
                "Shooter " + label.charAt(0),
                "tgt=%.0f cur=%6.0f err=%+5.0f %s",
                s.targetRpm,
                s.rpm,
                s.errorRpm,
                s.ready ? "READY" : "spin-up"
        );

        telemetry.addData(
                "Shooter " + label.charAt(0) + " PID",
                "pwr=%.2f I=%.1f",
                s.power,
                s.pid.getIntegral()
        );
    }

    private static double stickCurve(double input) {

        if (Math.abs(input) < STICK_DEADBAND) {
            return 0;
        }

        double normalized =
                (Math.abs(input) - STICK_DEADBAND) / (1.0 - STICK_DEADBAND);

        return Math.signum(input) * Math.pow(normalized, STICK_EXPONENT);
    }

    private static double slew(double target, double lastOut, double maxStep) {

        if (Math.abs(target) <= Math.abs(lastOut) && target * lastOut >= 0) {
            return target;
        }

        double delta = clamp(target - lastOut, -maxStep, maxStep);

        return lastOut + delta;
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private static double wrapDegrees(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }
}
