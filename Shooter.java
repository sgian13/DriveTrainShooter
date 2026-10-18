package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Shooter {

    public static final double SHOOTER_TICKS_PER_REV    = 28;
    public static final double SHOOTER_MAX_RPM          = 6000;
    public static final double SHOOTER_KP               = 0.00030;
    public static final double SHOOTER_KI               = 0.00003;
    public static final double SHOOTER_KD               = 0.00001;
    public static final double SHOOTER_INTEGRAL_LIMIT   = 6000;
    public static final double SHOOTER_READY_ERROR_RPM  = 120.0;
    public static final boolean SHOOTER_DEBUG_TELEMETRY = false;

    public static double maxTicksPerSec() {
        return SHOOTER_MAX_RPM / 60.0 * SHOOTER_TICKS_PER_REV;
    }

    public final DcMotorEx motor;
    public final double targetRpm;
    public final double targetTicksPerSec;
    public final ElapsedTime timer = new ElapsedTime();

    // ---- PID state (inlined, no separate GroboControl) ----
    private final double kF;
    private double integral = 0;
    private double lastError = 0;
    private double smoothedDerivative = 0;
    private boolean firstRun = true;

    // Telemetry snapshot του integral
    public double integralForTelemetry = 0;

    // ---- Existing shooter state ----
    public boolean on = false;
    public boolean ready = false;
    public boolean rumbleDone = false;

    public double rpm = 0;
    public double errorRpm = 0;
    public double power = 0;

    public double peakRpm = 0;
    public double peakCurrent = 0;
    public double spinUpTime = -1;

    public Shooter(DcMotorEx motor, double targetRpm, double kF, DcMotorSimple.Direction direction) {
        this.motor = motor;
        this.targetRpm = targetRpm;
        this.targetTicksPerSec = targetRpm / 60.0 * SHOOTER_TICKS_PER_REV;
        this.kF = kF;

        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setPower(0);
    }

    // ---- PID helpers (inlined) ----
    private void pidReset() {
        integral = 0;
        lastError = 0;
        smoothedDerivative = 0;
        firstRun = true;
        integralForTelemetry = 0;
    }

    private double pidCalculate(double targetTicksPerSec, double currentTicksPerSec, double dtVal) {
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
        double pTerm = SHOOTER_KP * error;
        double dTerm = SHOOTER_KD * derivative;

        double candidateIntegral = clamp(
                integral + error * safeDt,
                -SHOOTER_INTEGRAL_LIMIT,
                SHOOTER_INTEGRAL_LIMIT
        );

        double unsaturated = feedForward + pTerm + SHOOTER_KI * candidateIntegral + dTerm;

        boolean saturatedHigh = unsaturated > 1.0 && error > 0;
        boolean saturatedLow  = unsaturated < 0.0 && error < 0;

        if (!saturatedHigh && !saturatedLow) {
            integral = candidateIntegral;
        }

        integralForTelemetry = integral;

        double output = feedForward + pTerm + SHOOTER_KI * integral + dTerm;
        return clamp(output, 0.0, 1.0);
    }

    // ---- Public API ----
    private void resetState() {
        pidReset();
        timer.reset();
        ready = false;
        rumbleDone = false;
        peakRpm = 0;
        peakCurrent = 0;
        spinUpTime = -1;
    }

    public void toggle() {
        on = !on;
        resetState();
    }

    public void stop() {
        on = false;
        resetState();
        motor.setPower(0);
    }

    public boolean update(boolean killSwitch, double dt) {
        double ticksPerSec = Math.abs(motor.getVelocity());
        rpm = ticksPerSec * 60.0 / SHOOTER_TICKS_PER_REV;
        errorRpm = targetRpm - rpm;

        boolean justReady = false;

        if (!killSwitch && on) {
            power = pidCalculate(targetTicksPerSec, ticksPerSec, dt);
            motor.setPower(power);

            ready = Math.abs(errorRpm) <= SHOOTER_READY_ERROR_RPM;

            if (ready && !rumbleDone) {
                rumbleDone = true;
                spinUpTime = timer.seconds();
                justReady = true;
            }

            if (rpm > peakRpm) peakRpm = rpm;

            if (SHOOTER_DEBUG_TELEMETRY) {
                double current = motor.getCurrent(CurrentUnit.AMPS);
                if (current > peakCurrent) peakCurrent = current;
            }
        } else {
            power = 0;
            motor.setPower(0);
            pidReset();
            ready = false;
        }

        return justReady;
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}