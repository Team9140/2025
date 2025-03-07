package org.team9140.frc2025;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.team9140.frc2025.generated.TunerConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
        public static final int MANIPULATOR_MOTOR = 0;
        public static final int FUNNEL_MOTOR = 13;
        public static final int CLIMBER_MOTOR = 12;
    }

    public static class Drive {
        public static final LinearVelocity SPEED_AT_12_VOLTS = TunerConstants.kSpeedAt12Volts.times(0.8);
        public static final LinearVelocity MIN_TRANSLATIONAL_SPEED = MetersPerSecond.of(0.0375);
        public static final LinearVelocity MIN_TRANSLATIONAL_SPEED_TELEOP = MetersPerSecond.of(0.01);
        public static final AngularVelocity MIN_ROTATIONAL_SPEED = DegreesPerSecond.of(1);
        public static final AngularVelocity MIN_ROTATIONAL_SPEED_TELEOP = DegreesPerSecond.of(0.1);
        public static final AngularVelocity MAX_ROTATIONAL_RATE = RotationsPerSecond.of(2);
    }

    public static class FieldItemPoses {
        public static final Pose2d REEF_BLUE = new Pose2d(4.48945, 4.0259, new Rotation2d());
        public static final Pose2d REEF_RED = new Pose2d(13.058775, 4.0259, new Rotation2d());
    }

    public static final Distance REEF_RADIUS = Meters.of(1.5);
    public static final Transform2d HORIZONTAL_BRANCH_DISTANCE_FROM_CENTER = new Transform2d(Meters.of(0),
            Meters.of(0.25), new Rotation2d());

    public static class Funnel {

        public static final Current STATOR_LIMIT = Amps.of(20);
        public static final Voltage INTAKE_VOLTAGE = Volts.of(6);

    }

    public static class Manipulator {
        public static final double HOLD_VOLTAGE_ALGAE = 0;

        public static final double INTAKE_VOLTAGE_CORAL = 6;
        public static final double INTAKE_VOLTAGE_ALGAE = 10;

        public static final double OUTTAKE_VOLTAGE_CORAL = 6;
        public static final double OUTTAKE_VOLTAGE_ALGAE = -10;

        public static final Measure<DistanceUnit> CORAL_DISTANCE = Centimeters.of(10);
        public static final Time INTAKE_CORAL_TIME = Seconds.of(1);

        public static final int MIN_SIGNAL_STRENGTH = 5000;
        public static final double PROXIMITY_HYSTERESIS = 0.05;
        public static final double PROXIMITY_THRESHOLD = 0.4;
        public static final double FORWARD_AUTOSET = 0.0;

        public static final Current MANIPULATOR_PEAK_CURRENT_LIMIT = Amps.of(30);
        public static final Time MANIPULATOR_PEAK_CURRENT_DURATION = Milliseconds.of(100.0);
        public static final Current MANIPULATOR_CONTINUOUS_CURRENT_LIMIT = Amps.of(10);
    }

    public static final class Elevator {
        public static final Mass mass = Pounds.of(15.0);

        public static final Distance MIN_HEIGHT = Inches.of(0);
        public static final Distance MAX_HEIGHT = Inches.of(90);

        public static final Current STATOR_LIMIT = Amps.of(100.0);

        public static final double GEAR_RATIO = 60.0 / 12.0;
        public static final Distance SPOOL_RADIUS = Inches.of(0.75);
        public static final Distance SPOOL_CIRCUMFERENCE = SPOOL_RADIUS.times(Math.PI * 2.0);

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond
                .of(Meters.of(20.0).div(SPOOL_CIRCUMFERENCE).magnitude());
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond
                .of(Meters.of(80.0).div(SPOOL_CIRCUMFERENCE).magnitude());

        public static Angle ElevatorAngle = Degrees.of(80);

        public static Distance BOTTOM = Inches.of(0);

    }
}
