//ΒΑΣΙΚΟ DRIVETRAIN ΠΡΟΓΡΑΜΜΑ
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

// =========================
// ΚΟΥΜΠΙΑ (gamepad1):
//   Αριστερό stick -> κίνηση (μπρος/πίσω/strafe)
//   Δεξί stick (x) -> στροφή
//   LB      -> SHOOTER LEFT ON/OFF + ΠΑΡΑΚΟΛΟΥΘΗΣΗ (ticks, rpm)
//   RB      -> SHOOTER RIGHT ON/OFF
//   B       -> SLOW MODE ON/OFF
//   Y       -> FIELD CENTRIC / ROBOT CENTRIC
//   X       -> INTAKE ON/OFF
//   D-PAD   -> UP: servo1, DOWN: servo2, RIGHT: servo3, LEFT: servo4
// =========================

@TeleOp(name = "Grobo-DriveTrain_Shooter_Servo")
public class DriveTrainServoShooter extends LinearOpMode {

    // =========================
    // ΡΥΘΜΙΣΕΙΣ (άλλαξέ τες εδώ)
    // =========================

    // Ισχύς των shooter όταν είναι ON
    private static final double SHOOTER_POWER = 1.0;

    // Ticks ανά περιστροφή του encoder (άλλαξέ το ανάλογα με το motor σου)
    private static final double SHOOTER_TICKS_PER_REV = 28;
    private static final double INTAKE_TICKS_PER_REV = 28;

    // Θεωρητικό max RPM του shooter motor (χωρίς φορτίο) και ποσοστό για το "spin-up"
    private static final double SHOOTER_MAX_RPM = 6000;
    private static final double SPIN_UP_FRACTION = 0.95;

    // true -> δείχνει στο telemetry peak RPM, peak ρεύμα και spin-up time του shooter left
    private static final boolean SHOOTER_DEBUG_TELEMETRY = false;

    // Θέσεις servo
    private static final double SERVO_REST = 0.1;
    private static final double SERVO_ACTIVE = 0.9;

    // Slow mode (μπρος/πίσω/strafe και στροφή)
    private static final double SLOW_DRIVE = 0.30;
    private static final double SLOW_TURN = 0.30;

    @Override
    public void runOpMode() {

        // =========================
        // ΚΑΤΑΣΤΑΣΕΙΣ ΚΟΥΜΠΙΩΝ (toggles)
        // =========================

        boolean fieldCentric = false;
        boolean lastY = false;

        boolean intakeOn = false;
        boolean lastX = false;

        boolean shooterLeftOn = false;
        boolean lastLB = false;

        boolean shooterRightOn = false;
        boolean lastRB = false;

        boolean slowMode = false;
        boolean lastB = false;

        boolean servo1Active = false;
        boolean servo2Active = false;
        boolean servo3Active = false;
        boolean servo4Active = false;

        boolean lastUp = false;
        boolean lastDown = false;
        boolean lastRight = false;
        boolean lastLeft = false;

        // --- Παρακολούθηση shooter left ---
        // Χρόνος από τη στιγμή που άνοιξε το shooter
        ElapsedTime shooterLeftTimer = new ElapsedTime();
        // Μέγιστα που έχουν καταγραφεί σε αυτό το ON
        double shooterLeftPeakRpm = 0;
        double shooterLeftPeakCurrent = 0;
        // Χρόνος που έφτασε το SPIN_UP_FRACTION του SHOOTER_MAX_RPM (-1 = δεν έφτασε ακόμα)
        double shooterLeftSpinUpTime = -1;


        // =========================
        // MOTORS (Control Hub)
        // =========================

        Motor frontLeft = new Motor(hardwareMap, "leftFMotor");
        Motor frontRight = new Motor(hardwareMap, "rightFMotor");
        Motor backLeft = new Motor(hardwareMap, "leftBMotor");
        Motor backRight = new Motor(hardwareMap, "rightBMotor");


        // =========================
        // INTAKE MOTOR
        // =========================

        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0);


        // =========================
        // SHOOTER MOTORS
        // =========================

        DcMotorEx shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        DcMotorEx shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        // ΠΡΟΣΟΧΗ: και τα δύο είναι REVERSE. Αν τα motors είναι τοποθετημένα
        // αντικριστά (καθρέφτης), το ένα πρέπει να είναι FORWARD. Έλεγξέ το στο ρομπότ.
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // RUN_WITHOUT_ENCODER = το power 1.0 δίνει πλήρη τάση στο motor (χωρίς velocity PID).
        // Το encoder συνεχίζει να διαβάζει ticks/RPM κανονικά.
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setPower(0);
        shooterRight.setPower(0);


        // =========================
        // SERVOS
        // =========================

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


        // =========================
        // MECANUM DRIVE
        // =========================

        MecanumDrive drive = new MecanumDrive(
                frontLeft,
                frontRight,
                backLeft,
                backRight
        );


        // =========================
        // IMU
        // =========================

        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Προσανατολισμός του Control Hub πάνω στο ρομπότ - έλεγξέ τον αν το heading βγαίνει λάθος
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );

        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();


        // =========================
        // READY
        // =========================

        telemetry.addData("Status", "Ready");
        telemetry.addData("Drive Mode", "ROBOT CENTRIC");
        telemetry.addData("Slow Mode (B)", "OFF");
        telemetry.addData("Intake (X)", "OFF");
        telemetry.addData("Shooter Left (LB)", "OFF");
        telemetry.addData("Shooter Right (RB)", "OFF");
        telemetry.update();

        waitForStart();


        // =========================
        // MAIN LOOP
        // =========================

        while (opModeIsActive()) {

            // --- FIELD / ROBOT CENTRIC (Y) ---
            if (gamepad1.y && !lastY) {
                fieldCentric = !fieldCentric;

                // Reset yaw όταν μπαίνουμε σε Field Centric
                if (fieldCentric) {
                    imu.resetYaw();
                }
            }
            lastY = gamepad1.y;


            // --- INTAKE ON / OFF (X) ---
            if (gamepad1.x && !lastX) {
                intakeOn = !intakeOn;
            }
            lastX = gamepad1.x;

            intakeMotor.setPower(intakeOn ? 1.0 : 0.0);


            // --- SHOOTER LEFT ON / OFF (LB) ---
            if (gamepad1.left_bumper && !lastLB) {
                shooterLeftOn = !shooterLeftOn;

                // Όταν ανοίγει: μηδένισε τα στατιστικά και ξεκίνα χρονόμετρο
                if (shooterLeftOn) {
                    shooterLeftTimer.reset();
                    shooterLeftPeakRpm = 0;
                    shooterLeftPeakCurrent = 0;
                    shooterLeftSpinUpTime = -1;
                }
            }
            lastLB = gamepad1.left_bumper;

            shooterLeft.setPower(shooterLeftOn ? SHOOTER_POWER : 0.0);


            // --- SHOOTER RIGHT ON / OFF (RB) ---
            if (gamepad1.right_bumper && !lastRB) {
                shooterRightOn = !shooterRightOn;
            }
            lastRB = gamepad1.right_bumper;

            shooterRight.setPower(shooterRightOn ? SHOOTER_POWER : 0.0);


            // --- SERVOS (D-PAD) ---
            if (gamepad1.dpad_up && !lastUp) {
                servo1Active = !servo1Active;
            }
            lastUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !lastDown) {
                servo2Active = !servo2Active;
            }
            lastDown = gamepad1.dpad_down;

            if (gamepad1.dpad_right && !lastRight) {
                servo3Active = !servo3Active;
            }
            lastRight = gamepad1.dpad_right;

            if (gamepad1.dpad_left && !lastLeft) {
                servo4Active = !servo4Active;
            }
            lastLeft = gamepad1.dpad_left;

            servo1.setPosition(servo1Active ? SERVO_ACTIVE : SERVO_REST);
            servo2.setPosition(servo2Active ? SERVO_ACTIVE : SERVO_REST);
            servo3.setPosition(servo3Active ? SERVO_ACTIVE : SERVO_REST);
            servo4.setPosition(servo4Active ? SERVO_ACTIVE : SERVO_REST);


            // --- SLOW MODE (B) ---
            // Κάθε πάτημα του B αλλάζει ON <-> OFF:
            //   ON : αργή κίνηση -> SLOW_DRIVE, αργή στροφή -> SLOW_TURN
            //   OFF: κανονική ταχύτητα 100%
            if (gamepad1.b && !lastB) {
                slowMode = !slowMode;
            }
            lastB = gamepad1.b;

            double driveSpeed = slowMode ? SLOW_DRIVE : 1.0;
            double turnSpeed = slowMode ? SLOW_TURN : 1.0;


            // --- JOYSTICKS ---
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            double strafe = x * driveSpeed;
            double forward = y * driveSpeed;
            double turn = rx * turnSpeed;


            // --- DRIVE ---
            // Το heading διαβάζεται μία φορά ανά loop (χρησιμοποιείται και στο telemetry)
            double heading = 0;

            if (fieldCentric) {
                heading = imu
                        .getRobotYawPitchRollAngles()
                        .getYaw(AngleUnit.DEGREES);

                drive.driveFieldCentric(strafe, forward, turn, heading);
            } else {
                drive.driveRobotCentric(strafe, forward, turn);
            }


            // --- SHOOTER LEFT - ΥΠΟΛΟΓΙΣΜΟΙ ΠΑΡΑΚΟΛΟΥΘΗΣΗΣ ---
            double slRpm = Math.abs(shooterLeft.getVelocity()) * 60.0 / SHOOTER_TICKS_PER_REV;
            int slTicks = shooterLeft.getCurrentPosition();

            if (shooterLeftOn) {

                if (slRpm > shooterLeftPeakRpm) {
                    shooterLeftPeakRpm = slRpm;
                }

                // Το ρεύμα διαβάζεται μόνο όταν χρειάζεται (επιπλέον διάβασμα από το hub)
                if (SHOOTER_DEBUG_TELEMETRY) {
                    double slCurrent = shooterLeft.getCurrent(CurrentUnit.AMPS);
                    if (slCurrent > shooterLeftPeakCurrent) {
                        shooterLeftPeakCurrent = slCurrent;
                    }
                }

                // "Έφτασε σε ταχύτητα" όταν περάσει το SPIN_UP_FRACTION του θεωρητικού max.
                // Με φορτίο μπορεί να μη φτάσει ποτέ - τότε ρύθμισε το SPIN_UP_FRACTION.
                if (shooterLeftSpinUpTime < 0 && slRpm >= SPIN_UP_FRACTION * SHOOTER_MAX_RPM) {
                    shooterLeftSpinUpTime = shooterLeftTimer.seconds();
                }
            }

            double srRpm = Math.abs(shooterRight.getVelocity()) * 60.0 / SHOOTER_TICKS_PER_REV;


            // =========================
            // TELEMETRY
            // =========================

            telemetry.addData("Drive Mode", fieldCentric ? "FIELD CENTRIC" : "ROBOT CENTRIC");

            telemetry.addData("Slow Mode (B)", slowMode ? "ON - 30%" : "OFF - 100%");

            telemetry.addData("Intake (X)", intakeOn ? "ON" : "OFF");

            double intakeRpm = Math.abs(intakeMotor.getVelocity()) * 60.0 / INTAKE_TICKS_PER_REV;
            telemetry.addData("Intake RPM", "%.0f", intakeRpm);

            telemetry.addData("Shooter Left (LB)", shooterLeftOn ? "ON" : "OFF");
            telemetry.addData("Shooter Left ticks", slTicks);
            telemetry.addData("Shooter Left RPM", "%.0f", slRpm);

            if (SHOOTER_DEBUG_TELEMETRY) {
                telemetry.addData("Shooter Left peak RPM", "%.0f", shooterLeftPeakRpm);
                telemetry.addData("Shooter Left peak A", "%.2f", shooterLeftPeakCurrent);
                telemetry.addData("Shooter Left spin-up (s)", "%.2f", shooterLeftSpinUpTime);
            }

            telemetry.addData("Shooter Right (RB)", shooterRightOn ? "ON" : "OFF");
            telemetry.addData("Shooter Right RPM", "%.0f", srRpm);

            // Το pos είναι η τελευταία θέση που ΣΤΑΛΘΗΚΕ στο servo (όχι ανάγνωση από αισθητήρα)
            telemetry.addData("Servo1 (UP)", "%s  pos=%.2f", servo1Active ? "ACTIVE" : "REST", servo1.getPosition());
            telemetry.addData("Servo2 (DOWN)", "%s  pos=%.2f", servo2Active ? "ACTIVE" : "REST", servo2.getPosition());
            telemetry.addData("Servo3 (RIGHT)", "%s  pos=%.2f", servo3Active ? "ACTIVE" : "REST", servo3.getPosition());
            telemetry.addData("Servo4 (LEFT)", "%s  pos=%.2f", servo4Active ? "ACTIVE" : "REST", servo4.getPosition());

            if (fieldCentric) {
                telemetry.addData("Heading", "%.1f°", heading);
            }

            telemetry.update();
        }

        // Σταμάτα τα motors όταν τελειώσει το OpMode
        drive.stop();
        intakeMotor.setPower(0);
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }
}
