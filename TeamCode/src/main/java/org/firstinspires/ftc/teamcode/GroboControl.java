
package org.firstinspires.ftc.teamcode;

public final class GroboControl {

    private GroboControl() {
        // Δε χρειάζεται να δημιουργούμε αντικείμενα.
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static double stickCurve(
            double input, double deadband, double exponent) {

        double value = clamp(input, -1.0, 1.0);
        double magnitude = Math.abs(value);

        if (magnitude <= deadband) {
            return 0.0;
        }

        double normalized =
                (magnitude - deadband) / (1.0 - deadband);

        return Math.copySign(
                Math.pow(normalized, exponent), value);
    }

    public static double slew(
            double target, double previous,
            double ratePerSecond, double dt) {

        double maxChange = ratePerSecond * Math.max(0.0, dt);
        return previous + clamp(
                target - previous, -maxChange, maxChange);
    }

    public static double wrapDegrees(double degrees) {
        while (degrees > 180.0) degrees -= 360.0;
        while (degrees < -180.0) degrees += 360.0;
        return degrees;
    }

    public static class VelocityPIDF {

        private final double kP;
        private final double kI;
        private final double kD;
        private final double kF;
        private final double integralLimit;
        private final double integralDeadzone;

        private double integral = 0.0;
        private double previousError = 0.0;
        private boolean firstCalculation = true;

        public VelocityPIDF(
                double kP, double kI, double kD, double kF,
                double integralLimit, double integralDeadzone) {

            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            this.integralLimit = Math.abs(integralLimit);
            this.integralDeadzone = Math.abs(integralDeadzone);
        }

        public double calculate(
                double target, double current, double dt) {

            if (dt <= 0.0 || Double.isNaN(dt)
                    || Double.isInfinite(dt)) {
                return kF * target + kP * (target - current)
                        + kI * integral;
            }

            double error = target - current;

            if (Math.abs(error) > integralDeadzone) {
                integral += error * dt;
                integral = clamp(
                        integral, -integralLimit, integralLimit);
            }

            double derivative = 0.0;

            if (!firstCalculation) {
                derivative = (error - previousError) / dt;
            }

            previousError = error;
            firstCalculation = false;

            return kF * target
                    + kP * error
                    + kI * integral
                    + kD * derivative;
        }

        public void reset() {
            integral = 0.0;
            previousError = 0.0;
            firstCalculation = true;
        }

        public double getIntegral() {
            return integral;
        }
    }
}
