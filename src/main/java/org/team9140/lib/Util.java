package org.team9140.lib;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

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

    public static final Distance TRANSLATION_E = Inches.of(1.5);
    public static final Angle ROTATION_E = Degrees.of(5);

    public static boolean epsilonEquals(Pose2d a, Pose2d b) {
        boolean transValid = a.getTranslation().getDistance(b.getTranslation()) < TRANSLATION_E.in(Meters);
        boolean rotValid = rotationEpsilonEquals(
                a.getRotation(), b.getRotation(), ROTATION_E.in(Radians));

        return transValid && rotValid;
    }

    public static boolean rotationEpsilonEquals(Rotation2d a, Rotation2d b, double epsilon) {
        return Math.abs(MathUtil.angleModulus(a.minus(b).getRadians())) <= epsilon;
        // final double tau = 2 * Math.PI;
        // double theta_a = a.getRadians() < 0 ? tau - (-a.getRadians() % tau):
        // a.getRadians() % tau;
        // double theta_b = b.getRadians() < 0 ? tau - (-b.getRadians() % tau):
        // b.getRadians() % tau;
        // return epsilonEquals(theta_a, theta_b, epsilon);
    }

    public static boolean rotationEpsilonEquals(Rotation2d a, Rotation2d b) {
        return rotationEpsilonEquals(a, b, Math.toRadians(5.0));
    }

    public static boolean epsilonEquals(Translation2d a, Translation2d b, double epsilon) {
        return epsilonEquals(a.getX(), b.getX(), epsilon) && epsilonEquals(a.getY(), b.getY(), epsilon);
    }

    public static boolean epsilonEquals(Translation2d a, Translation2d b) {
        return epsilonEquals(a, b, EPSILON);
    }
}
