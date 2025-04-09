package org.team9140.frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public class Constants {
    public static final Time LOOP_PERIOD = Milliseconds.of(20.0);

    public static class Ports {
        public static final int ELEVATOR_MOTOR_LEFT = 10;
        public static final int ELEVATOR_MOTOR_RIGHT = 11;
        public static final int MANIPULATOR_MOTOR = 4;
        public static final int FUNNEL_MOTOR = 13;
        public static final int CLIMBER_MOTOR_RIGHT = 14;
        public static final int CLIMBER_MOTOR_LEFT = 12;
        public static final int CANDLE_id = 0;
    }

    public static class Drive {
        public static final LinearVelocity SPEED_AT_12_VOLTS = MetersPerSecond.of(4.48);
        public static final LinearVelocity MAX_teleop_velocity = SPEED_AT_12_VOLTS.times(0.8);
        public static final AngularVelocity MAX_teleop_rotation = RotationsPerSecond.of(1);

        public static final LinearVelocity MIN_TRANSLATIONAL_SPEED = MetersPerSecond.of(0.06);
        public static final LinearVelocity MIN_TRANSLATIONAL_SPEED_TELEOP = MetersPerSecond.of(0.02);
        public static final AngularVelocity MIN_ROTATIONAL_SPEED = DegreesPerSecond.of(3);
        public static final AngularVelocity MIN_ROTATIONAL_SPEED_TELEOP = DegreesPerSecond.of(3);

        public static final double X_CONTROLLER_P = 2.5 * 3.141592653589793238462643383279502884197169399375;
        public static final double X_CONTROLLER_I = 0.0;
        public static final double X_CONTROLLER_D = 0.015; // TODO: Raise value
        public static final double Y_CONTROLLER_P = X_CONTROLLER_P;
        public static final double Y_CONTROLLER_I = X_CONTROLLER_I;
        public static final double Y_CONTROLLER_D = X_CONTROLLER_D;
        public static final double HEADING_CONTROLLER_P = 8.0; // 8.0
        public static final double HEADING_CONTROLLER_I = 0.0;
        public static final double HEADING_CONTROLLER_D = 0.03; // 0.04

        public static final Time REACHEDPOSE_DEBOUNCE = Seconds.of(0.5);
    }

    public static class FieldItemPoses {
        public static final Pose2d REEF_BLUE = new Pose2d(4.48945, 4.0259, new Rotation2d());
        public static final Pose2d REEF_RED = new Pose2d(13.058775, 4.0259, new Rotation2d());
    }
            
    public static class Funnel {
        public static final Current STATOR_LIMIT = Amps.of(70);
        public static final Current SUPPLY_LIMIT = Amps.of(30);
        public static final Voltage INTAKE_VOLTAGE = Volts.of(12);
        public static final Voltage UNSTICK_VOLTAGE = Volts.of(-8);
    }

    public static class Manipulator {
        public static final double HOLD_VOLTAGE_ALGAE = 4.0;

        public static final double INTAKE_VOLTAGE_CORAL = 7.0;
        public static final double INTAKE_VOLTAGE_ALGAE = 10;

        public static final double OUTTAKE_VOLTAGE_CORAL = 8;
        public static final double OUTTAKE_VOLTAGE_ALGAE = -10;

        // public static final Measure<DistanceUnit> CORAL_DISTANCE = Centimeters.of(10);
        // public static final Time INTAKE_CORAL_TIME = Seconds.of(1);

        // public static final int MIN_SIGNAL_STRENGTH = 5000;
        // public static final double PROXIMITY_HYSTERESIS = 0.05;
        // public static final double PROXIMITY_THRESHOLD = 0.4;
        // public static final double FORWARD_AUTOSET = 0.0;

        public static final Current MANIPULATOR_PEAK_CURRENT_LIMIT = Amps.of(30);
        public static final Time MANIPULATOR_PEAK_CURRENT_DURATION = Milliseconds.of(500.0);
        public static final Current MANIPULATOR_CONTINUOUS_CURRENT_LIMIT = Amps.of(20);
    }

    public static final class Elevator {
        public static final Mass mass = Pounds.of(0.1);

        public static final Current STATOR_LIMIT = Amps.of(60.0);

        public static final double GEAR_RATIO = 60.0 / 12.0;
        public static final Distance SPOOL_RADIUS = Inches.of(0.75);
        public static final Distance SPOOL_CIRCUMFERENCE = SPOOL_RADIUS.times(Math.PI * 2.0);

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond
                .of(Meters.of(3.0).div(SPOOL_CIRCUMFERENCE).magnitude());
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond
                .of(Meters.of(5.0).div(SPOOL_CIRCUMFERENCE).magnitude());

        public static Angle ElevatorAngle = Degrees.of(80);

        public static final Distance MIN_HEIGHT = Inches.of(0);
        public static final Distance MAX_HEIGHT = Inches.of(90);

        public static final Measure<DistanceUnit> POSITION_epsilon = Inches.of(0.75);

        public static Distance STOW_height = Inches.of(0.5);
        public static Distance L1_coral_height = Inches.of(24);
        public static Distance L2_coral_height = Inches.of(31);
        public static Distance L3_coral_height = Inches.of(47);
        public static Distance L4_coral_height = Inches.of(72.0);

        public static Distance L2_ALGAE_height = Inches.of(24); // TODO: Actual Value
        public static Distance L3_ALGAE_height = Inches.of(39);

        public static Distance NET_HEIGHT = Inches.of(80); // TODO: Actual Value

        public static Distance SOFT_LIMIT = Inches.of(81);
    }

    public static final class Climber {
        public static final Current STATOR_LIMIT = Amps.of(80.0);
        public static final double GEAR_RATIO = 5.0 * 5.0 * 4.0 * 4.0;
        public static final Angle SOFT_LIMIT_LOWER = Radians.of(0.0);
        public static final Angle SOFT_LIMIT_HIGHER = Rotations.of(0.7);

        public static final Voltage MAX_OUTPUT = Volts.of(12.0);
        public static final Voltage PREP_VOLTAGE = Volts.of(10.0);
        public static final Angle PREPPED_POSITION = Rotations.of(0.46);
    }

    public enum ElevatorSetbacks {
        NONE(Inches.of(0)),
        L1(Inches.of(18.0)),
        L2(Inches.of(18.0)),
        L3(Inches.of(18.0 + 4.0)),
        L4(Inches.of(18.0 + 9.0)),
        ALGAE_L2(Inches.of(18.0)),
        ALGAE_L3(Inches.of(18.0 + 3.0));

        ElevatorSetbacks(Distance setback) {
            this.setbackinator = new Transform2d(setback.unaryMinus(), Meters.of(0), new Rotation2d());
        }

        private final Transform2d setbackinator;

        public Transform2d getSetbackinator() {
            return setbackinator;
        }
    }


    public static final class AutoAlign {
        public static final Distance REEF_RADIUS = Feet.of(5).plus(Inches.of(5.5)).div(2);

        public static final Transform2d HORIZONTAL_BRANCH_DISTANCE_FROM_CENTER = new Transform2d(Meters.of(0),
                Inches.of(13).div(2), new Rotation2d());

        // gap between two reef branches on the same face
        private static final Distance reefBranchGap = Inches.of(13.5);
        // how many meters straight out from apriltag should center of robot be for L1 / L2 / L3 / L4
        private static final Distance L1setback = Inches.of(18.0);
        private static final Distance L2setback = Inches.of(18.0); // wall bump
        private static final Distance L3setback = Inches.of(18.0 + 4.0); // 4 inch behind wall
        private static final Distance L4setback = Inches.of(18.0 + 9.0); // 11 inch behind wall

        // from the tag perspective, how far OUT (+x) and LEFT (+y) should the robot be to score?
        // rotate these around by a tag's orientation on the field then add to tag pose to get target pose for any reef spot
        public static final Translation2d leftBranchOffset_L1 = new Translation2d(L1setback, reefBranchGap.times(-0.5));
        public static final Translation2d rightBranchOffset_L1 = new Translation2d(L1setback, reefBranchGap.times(0.5));

        public static final Translation2d leftBranchOffset_L2 = new Translation2d(L2setback, reefBranchGap.times(-0.5));
        public static final Translation2d rightBranchOffset_L2 = new Translation2d(L2setback, reefBranchGap.times(0.5));

        public static final Translation2d leftBranchOffset_L3 = new Translation2d(L3setback, reefBranchGap.times(-0.5));
        public static final Translation2d rightBranchOffset_L3 = new Translation2d(L3setback, reefBranchGap.times(0.5));

        public static final Translation2d leftBranchOffset_L4 = new Translation2d(L4setback, reefBranchGap.times(-0.5));
        public static final Translation2d rightBranchOffset_L4 = new Translation2d(L4setback, reefBranchGap.times(0.5));
    }

}
