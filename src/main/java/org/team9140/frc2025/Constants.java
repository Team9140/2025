package org.team9140.frc2025;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.team9140.frc2025.generated.TunerConstants;

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


public class Constants {

    public static final Time LOOP_PERIOD = Milliseconds.of(20.0);

    public static class Ports {
        public static final int ELEVATOR_MOTOR_LEFT = 10;
        public static final int ELEVATOR_MOTOR_RIGHT = 11;
        public static final int MANIPULATOR_MOTOR = 4;
        public static final int FUNNEL_MOTOR = 13;
        public static final int CLIMBER_MOTOR = 12;
        public static final int CANDLE_id = 0;
    }

    public static class Drive {
        public static final LinearVelocity MAX_teleop_velocity = TunerConstants.kSpeedAt12Volts.times(0.8);
        public static final AngularVelocity MAX_teleop_rotation = RotationsPerSecond.of(1);

        public static final LinearVelocity MIN_TRANSLATIONAL_SPEED = MetersPerSecond.of(0.0375);
        public static final LinearVelocity MIN_TRANSLATIONAL_SPEED_TELEOP = MetersPerSecond.of(0.01);
        public static final AngularVelocity MIN_ROTATIONAL_SPEED = DegreesPerSecond.of(1);
        public static final AngularVelocity MIN_ROTATIONAL_SPEED_TELEOP = DegreesPerSecond.of(0.1);
    }

    public static class FieldItemPoses {
        public static final Pose2d REEF_BLUE = new Pose2d(4.48945, 4.0259, new Rotation2d());
        public static final Pose2d REEF_RED = new Pose2d(13.058775, 4.0259, new Rotation2d());
    }

    public static final Distance REEF_RADIUS = Meters.of(1.5);
    public static final Transform2d HORIZONTAL_BRANCH_DISTANCE_FROM_CENTER = new Transform2d(Meters.of(0),
            Meters.of(0.25), new Rotation2d());

            
    public static class Funnel {
        public static final Current STATOR_LIMIT = Amps.of(70);
        public static final Current SUPPLY_LIMIT = Amps.of(30);
        public static final Voltage INTAKE_VOLTAGE = Volts.of(12);
    }

    public static class Manipulator {
        public static final double HOLD_VOLTAGE_ALGAE = 0;

        public static final double INTAKE_VOLTAGE_CORAL = 6;
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
        public static final Time MANIPULATOR_PEAK_CURRENT_DURATION = Milliseconds.of(100.0);
        public static final Current MANIPULATOR_CONTINUOUS_CURRENT_LIMIT = Amps.of(10);
    }

    public static final class Elevator {
        public static final Mass mass = Pounds.of(15.0);

        public static final Current STATOR_LIMIT = Amps.of(50.0);

        public static final double GEAR_RATIO = 60.0 / 12.0;
        public static final Distance SPOOL_RADIUS = Inches.of(0.75);
        public static final Distance SPOOL_CIRCUMFERENCE = SPOOL_RADIUS.times(Math.PI * 2.0);

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond
                .of(Meters.of(2.0).div(SPOOL_CIRCUMFERENCE).magnitude());
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond
                .of(Meters.of(4.0).div(SPOOL_CIRCUMFERENCE).magnitude());

        public static Angle ElevatorAngle = Degrees.of(80);

        public static final Distance MIN_HEIGHT = Inches.of(0);
        public static final Distance MAX_HEIGHT = Inches.of(90);

        public static final Measure<DistanceUnit> POSITION_epsilon = Inches.of(0.75);

        public static Distance STOW_height = MIN_HEIGHT;
        public static Distance L1_coral_height = Inches.of(24);
        public static Distance L2_coral_height = Inches.of(30);
        public static Distance L3_coral_height = Inches.of(46);
        public static Distance L4_coral_height = Inches.of(73.5);
    }

    public static final class AutoAlign {
        // gap between two reef branches on the same face
        private static final Distance reefBranchGap = Inches.of(13.0);
        // how many meters straight out from apriltag should center of robot be for L1 / L2 / L3 / L4
        private static final Distance L1setback = Inches.of(18.0);
        private static final Distance L2setback = Inches.of(18.0); // wall bump
        private static final Distance L3setback = Inches.of(18.0 + 4.0); // 4 inch behind wall
        private static final Distance L4setback = Inches.of(18.0 + 11.0); // 11 inch behind wall

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
