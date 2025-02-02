package org.team9140.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

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

    public static boolean rotationEpsilonEquals(Rotation2d a, Rotation2d b, double epsilon) {
        return Math.abs(MathUtil.angleModulus(a.minus(b).getRadians())) <= epsilon;
//        final double tau = 2 * Math.PI;
//        double theta_a = a.getRadians() < 0 ? tau - (-a.getRadians() % tau): a.getRadians() % tau;
//        double theta_b = b.getRadians() < 0 ? tau - (-b.getRadians() % tau): b.getRadians() % tau;
//        return epsilonEquals(theta_a, theta_b, epsilon);
    }

    public static boolean rotationEpsilonEquals(Rotation2d a, Rotation2d b) {
        return rotationEpsilonEquals(a, b, Math.toRadians(5.0));
    }
}
