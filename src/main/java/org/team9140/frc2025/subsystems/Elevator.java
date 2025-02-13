package org.team9140.frc2025.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.team9140.frc2025.Constants;

import static edu.wpi.first.units.Units.Meters;

// make a subsystem
public class Elevator implements Subsystem {
// two talonFXs (one main, one follower)     constants?
    TalonFX talonMain = new TalonFX(52);
    TalonFX talonFollower = new TalonFX(52);
    CANdi elevatorCandi = new CANdi(0);
    private final MotionMagicVoltage voltage;

    public Elevator() {
        // canDI
        var limits = new HardwareLimitSwitchConfigs();
        limits.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANdiS1;
        limits.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANdiS2;

        var feedback = new FeedbackConfigs();
        feedback.withSensorToMechanismRatio(Constants.Elevator.METERS_PER_MOTOR_ROTATION);

        // use as remote limit switch for main TalonFX motor
        talonMain.getConfigurator().apply(limits);

        // set up MotionMagicVoltage control request (see 2024 Arm.java for example)
        this.voltage = new MotionMagicVoltage(Constants.ArmPositions.INTAKE)
                .withEnableFOC(true)
                .withSlot(0)
                .withFeedForward(Constants.ArmPositions.FEED_FORWARD);
    }

// make up gear ratio for a conversion factor - units to radians
    FeedbackConfigs feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(Constants.ArmPositions.SENSOR_TO_MECHANISM_RATIO);

// config current limit
    CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.ArmPositions.MAX_CURRENT).withStatorCurrentLimitEnable(true);

// needs command to go to position
// take a Distance, not a double
    public Command toPostion(Distance distance) {
        distance.in(Meters);
        return this.runOnce(() -> this.voltage.withPosition(distance));
    }

    public Command disable() {
        return this.runOnce(() -> this.talonMain.setControl(new CoastOut()));
    }
}
