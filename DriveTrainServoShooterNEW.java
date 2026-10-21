
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

@TeleOp(name = "Grobo-DriveTrain_Shooter_ServoMAIN")
public class DriveTrainServoShooterNEW extends LinearOpMode {

    // SHOOTER TARGETS
    private static final double SHOOTER_LEFT_TARGET_RPM = 6000;
    private static final double SHOOTER_RIGHT_TARGET_RPM = 6000;
    private static final double SHOOTER_TICKS_PER_REV = 28;

    // DRIVE SETTINGS
    private static final double STICK_DEADBAND = 0.05;
    private static final double STICK_EXPONENT = 1.6;

    private static final double SLOW_DRIVE = 0.30;
    private static final double SLOW_TURN = 0.30;

    // HEADING HOLD: αρχικές τιμές για δοκιμή
    private static final boolean HEADING_HOLD_DEFAULT = true;
    private static final double HEADING_HOLD_KP = 0.012;
    private static final double HEADING_HOLD_MAX_POWER = 0.35;
    private static final double HEADING_HOLD_DEADBAND_DEG = 1.0;
    private static final double HEADING_HOLD_DIRECTION = 1.0;
    private static final double HEADING_HOLD_SETTLE_SEC = 0.25;

    // SMOOTH MOVEMENT
    private static final double DRIVE_SLEW_RATE_PER_SEC = 6.0;
    private static final double TURN_SLEW_RATE_PER_SEC = 8.0;

    // SERVOS
    private static final double SERVO_REST = 0.1;
    private static final double SERVO_ACTIVE = 0.9;

    @Override
    public void runOpMode() {

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        boolean fieldCentric = false;
        boolean intakeOn = false;
        boolean slowMode = false;
        boolean headingHold = HEADING_HOLD_DEFAULT;

        boolean[] servoActive = new boolean[4];

        Edge startEdge = new Edge();
        Edge yEdge = new Edge();
        Edge xEdge = new Edge();
        Edge bEdge = new Edge();
        Edge lbEdge = new Edge();
        Edge rbEdge = new Edge();
        Edge lsEdge = new Edge();

        Edge[] dpadEdges = {
                new Edge(), new Edge(), new Edge(), new Edge()
        };

        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime headingSettleTimer = new ElapsedTime();

        double lastLoopTime = 0;
        double lastStrafeCmd = 0;
        double lastForwardCmd = 0;
        double lastTurnCmd = 0;
        double holdHeadingDeg = 0;

        int telemetryCounter = 0;

        Motor frontLeft = null;
        Motor frontRight = null;
        Motor backLeft = null;
        Motor backRight = null;

        DcMotorEx intakeMotor = null;
        Shooter shooterLeft = null;
        Shooter shooterRight = null;
        IMU imu = null;

        Servo[] servos = new Servo[4];

        boolean hardwareInitSuccess = true;
        boolean driveOk = true;
        StringBuilder initErrorMsg = new StringBuilder();

        // DRIVE MOTORS
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

        MecanumDrive drive = null;

        if (driveOk) {
            drive = new MecanumDrive(
                    frontLeft, frontRight, backLeft, backRight
            );
        }

        // INTAKE
        try {
            intakeMotor = hardwareMap.get(
                    DcMotorEx.class, "intakeMotor"
            );

            intakeMotor.setDirection(
                    DcMotorSimple.Direction.REVERSE
            );
            intakeMotor.setMode(
                    DcMotor.RunMode.RUN_WITHOUT_ENCODER
            );
            intakeMotor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.BRAKE
            );
            intakeMotor.setPower(0);

        } catch (Exception e) {
            intakeMotor = null;
            hardwareInitSuccess = false;
            initErrorMsg.append("intakeMotor missing. ");
        }

        // SHOOTERS
        double kFeedForward = 1.0 / Shooter.maxTicksPerSec();

        try {
            shooterLeft = new Shooter(
                    hardwareMap.get(DcMotorEx.class, "shooterLeft"),
                    SHOOTER_LEFT_TARGET_RPM,
                    kFeedForward,
                    DcMotorSimple.Direction.REVERSE
            );
        } catch (Exception e) {
            shooterLeft = null;
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
            shooterRight = null;
            hardwareInitSuccess = false;
            initErrorMsg.append("shooterRight missing. ");
        }

        // SERVOS
        String[] servoNames = {
                "servo1", "servo2", "servo3", "servo4"
        };

        for (int i = 0; i < servos.length; i++) {
            try {
                servos[i] = hardwareMap.get(
                        Servo.class, servoNames[i]
                );

                servos[i].setDirection(Servo.Direction.FORWARD);
                servos[i].setPosition(SERVO_REST);

            } catch (Exception e) {
                servos[i] = null;
                hardwareInitSuccess = false;
                initErrorMsg.append(servoNames[i]).append(" missing. ");
            }
        }

        // IMU
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
            telemetry.addData("Status", "Ready");
        }

        telemetry.addData("Drive Mode", "ROBOT CENTRIC");
        telemetry.addData(
                "Shooter L Target", "%.0f RPM",
                SHOOTER_LEFT_TARGET_RPM
        );
        telemetry.addData(
                "Shooter R Target", "%.0f RPM",
                SHOOTER_RIGHT_TARGET_RPM
        );
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopAll(drive, intakeMotor, shooterLeft, shooterRight, servos);
            return;
        }

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

            // BUTTON EDGE DETECTION
            boolean startPressed = startEdge.pressed(gamepad1.start);
            boolean yPressed = yEdge.pressed(gamepad1.y);
            boolean xPressed = xEdge.pressed(gamepad1.x);
            boolean bPressed = bEdge.pressed(gamepad1.b);
            boolean lbPressed = lbEdge.pressed(gamepad1.left_bumper);
            boolean rbPressed = rbEdge.pressed(gamepad1.right_bumper);
            boolean lsPressed = lsEdge.pressed(
                    gamepad1.left_stick_button
            );

            boolean[] dpadPressed = {
                    dpadEdges[0].pressed(gamepad1.dpad_up),
                    dpadEdges[1].pressed(gamepad1.dpad_down),
                    dpadEdges[2].pressed(gamepad1.dpad_right),
                    dpadEdges[3].pressed(gamepad1.dpad_left)
            };

            // READ IMU
            double headingDeg = 0.0;

            if (imu != null) {
                try {
                    headingDeg = imu.getRobotYawPitchRollAngles()
                            .getYaw(AngleUnit.DEGREES);
                } catch (Exception ignored) {
                }
            }

            // START: RESET YAW
            if (startPressed && imu != null) {
                imu.resetYaw();
                headingDeg = 0.0;
                holdHeadingDeg = 0.0;
                headingSettleTimer.reset();
            }

            // MODE TOGGLES
            if (yPressed) {
                fieldCentric = !fieldCentric;
            }

            if (xPressed) {
                intakeOn = !intakeOn;
            }

            if (bPressed) {
                slowMode = !slowMode;
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

            // INTAKE
            if (intakeMotor != null) {
                intakeMotor.setPower(intakeOn ? 1.0 : 0.0);
            }

            // SERVOS
            for (int i = 0; i < servos.length; i++) {
                if (servos[i] != null) {
                    servos[i].setPosition(
                            servoActive[i] ? SERVO_ACTIVE : SERVO_REST
                    );
                }
            }

            // SHOOTER CONTROL
            boolean leftJustReady = false;
            boolean rightJustReady = false;

            if (shooterLeft != null) {
                leftJustReady = shooterLeft.update(false, dt);
            }

            if (shooterRight != null) {
                rightJustReady = shooterRight.update(false, dt);
            }

            if (leftJustReady || rightJustReady) {
                gamepad1.rumbleBlips(1);
            }

            // DRIVE INPUT
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

            boolean driverTurning =
                    Math.abs(rawRx) > STICK_DEADBAND;

            // HEADING HOLD
            if (imu == null || !headingHold || driverTurning) {
                holdHeadingDeg = headingDeg;
                headingSettleTimer.reset();

            } else if (headingSettleTimer.seconds()
                    < HEADING_HOLD_SETTLE_SEC) {

                holdHeadingDeg = headingDeg;
                turnCmd = 0;

            } else {
                double hErr = wrapDegrees(
                        holdHeadingDeg - headingDeg
                );

                if (Math.abs(hErr) > HEADING_HOLD_DEADBAND_DEG) {
                    double holdPower =
                            HEADING_HOLD_DIRECTION
                                    * HEADING_HOLD_KP * hErr;

                    turnCmd = clamp(
                            holdPower,
                            -HEADING_HOLD_MAX_POWER,
                            HEADING_HOLD_MAX_POWER
                    ) * turnSpeed;
                } else {
                    turnCmd = 0;
                }
            }

            // SMOOTH DRIVE COMMANDS
            double driveStep = DRIVE_SLEW_RATE_PER_SEC * dt;
            double turnStep = TURN_SLEW_RATE_PER_SEC * dt;

            strafeCmd = slew(strafeCmd, lastStrafeCmd, driveStep);
            forwardCmd = slew(forwardCmd, lastForwardCmd, driveStep);
            turnCmd = slew(turnCmd, lastTurnCmd, turnStep);

            lastStrafeCmd = strafeCmd;
            lastForwardCmd = forwardCmd;
            lastTurnCmd = turnCmd;

            // DRIVE
            if (drive != null) {
                if (fieldCentric && imu != null) {
                    drive.driveFieldCentric(
                            strafeCmd, forwardCmd, turnCmd, headingDeg
                    );
                } else {
                    drive.driveRobotCentric(
                            strafeCmd, forwardCmd, turnCmd
                    );
                }
            }

            // TELEMETRY
            if (++telemetryCounter >= 3) {
                telemetryCounter = 0;

                double intakeRpm = intakeMotor != null
                        ? Math.abs(intakeMotor.getVelocity())
                          * 60.0 / SHOOTER_TICKS_PER_REV
                        : 0.0;

                if (!hardwareInitSuccess) {
                    telemetry.addData(
                            "INIT ERROR", initErrorMsg.toString()
                    );
                }

                telemetry.addData("Loop Rate", "%.1f Hz", loopHz);
                telemetry.addData(
                        "Drive Mode",
                        fieldCentric && imu != null
                                ? "FIELD CENTRIC" : "ROBOT CENTRIC"
                );
                telemetry.addData(
                        "Slow Mode (B)", slowMode ? "ON - 30%" : "OFF"
                );
                telemetry.addData(
                        "Heading",
                        "%.1f deg (hold %.1f) %s",
                        headingDeg,
                        holdHeadingDeg,
                        headingHold && imu != null ? "HOLD ON" : "HOLD OFF"
                );
                telemetry.addData("Intake (X)", intakeOn ? "ON" : "OFF");
                telemetry.addData("Intake RPM", "%.0f", intakeRpm);

                if (shooterLeft != null) {
                    addShooterTelemetry("L (LB)", shooterLeft);
                }

                if (shooterRight != null) {
                    addShooterTelemetry("R (RB)", shooterRight);
                }

                String[] servoLabels = {
                        "UP", "DOWN", "RIGHT", "LEFT"
                };

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

        stopAll(drive, intakeMotor, shooterLeft, shooterRight, servos);
    }

    private void stopAll(
            MecanumDrive drive,
            DcMotorEx intakeMotor,
            Shooter shooterLeft,
            Shooter shooterRight,
            Servo[] servos
    ) {
        if (drive != null) {
            drive.stop();
        }

        if (intakeMotor != null) {
            intakeMotor.setPower(0);
        }

        if (shooterLeft != null) {
            shooterLeft.stop();
        }

        if (shooterRight != null) {
            shooterRight.stop();
        }

        for (Servo servo : servos) {
            if (servo != null) {
                servo.setPosition(SERVO_REST);
            }
        }
    }

    private void addShooterTelemetry(String label, Shooter s) {
        telemetry.addData(
                "Shooter " + label, s.on ? "ON" : "OFF"
        );

        telemetry.addData(
                "Shooter " + label.charAt(0),
                "tgt=%.0f cur=%.0f err=%+.0f %s",
                s.targetRpm,
                s.rpm,
                s.errorRpm,
                s.ready ? "READY" : "spin-up"
        );

        telemetry.addData(
                "Shooter " + label.charAt(0) + " PID",
                "pwr=%.2f I=%.1f",
                s.power,
                s.integralForTelemetry
        );
    }

    private static double stickCurve(double input) {
        if (Math.abs(input) < STICK_DEADBAND) {
            return 0;
        }

        double normalized =
                (Math.abs(input) - STICK_DEADBAND)
                        / (1.0 - STICK_DEADBAND);

        return Math.signum(input)
                * Math.pow(normalized, STICK_EXPONENT);
    }

    private static double slew(
            double target, double lastOut, double maxStep
    ) {
        if (Math.abs(target) <= Math.abs(lastOut)
                && target * lastOut >= 0) {
            return target;
        }

        double delta = clamp(
                target - lastOut, -maxStep, maxStep
        );

        return lastOut + delta;
    }

    private static double clamp(
            double v, double min, double max
    ) {
        return Math.max(min, Math.min(max, v));
    }

    private static double wrapDegrees(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }
}
