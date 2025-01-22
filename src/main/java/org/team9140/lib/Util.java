package org.team9140.lib;

public class Util {
    private final static double defaultDeadband = 0.12;
    private final static double EPSILON = 0.00000001;

    public static double applyDeadband(double in, double deadband) {
        if (Math.abs(in) < deadband) {
            return 0.0;
        } else if (in > 0) {
            return (in - deadband) / (1.0 - deadband);
        } else {
            return (in + deadband) / (1.0 - deadband);
        }
    }

    public static double applyDeadband(double in) {
        return applyDeadband(in, defaultDeadband);
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return Math.abs(a - b) < epsilon;
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }
}
