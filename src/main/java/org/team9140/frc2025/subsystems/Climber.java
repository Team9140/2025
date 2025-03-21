package org.team9140.frc2025.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.Command;
import org.team9140.frc2025.Constants;
import org.team9140.frc2025.Constants.Ports;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

public class Climber extends SubsystemBase {
    private final TalonFX rightMotor;
    private final TalonFX leftMotor;

    private final VoltageOut controller;
    
    private Climber() {
        this.rightMotor = new TalonFX(Ports.CLIMBER_MOTOR_RIGHT, "sigma");
        this.leftMotor = new TalonFX(Ports.CLIMBER_MOTOR_LEFT, "sigma");

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Climber.STATOR_LIMIT)
                .withStatorCurrentLimitEnable(true);

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.Climber.GEAR_RATIO);

        SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(Constants.Climber.SOFT_LIMIT_HIGHER)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Constants.Climber.SOFT_LIMIT_LOWER)
                .withReverseSoftLimitEnable(true);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withCurrentLimits(currentLimitsConfigs)
                .withFeedback(feedbackConfigs)
                .withMotorOutput(motorOutputConfigs)
                .withSoftwareLimitSwitch(softLimits);

        this.rightMotor.getConfigurator().apply(motorConfig);
        this.rightMotor.getConfigurator().apply(feedbackConfigs);
        this.rightMotor.setPosition(0.0);

        this.controller = new VoltageOut(0.0)
                .withEnableFOC(true);

        this.leftMotor.setControl(new Follower(this.rightMotor.getDeviceID(), true));
    }

    private static Climber instance;

    public static Climber getInstance() {
        return (instance == null) ? instance = new Climber() : instance;
    }

    public Command climb(Supplier<Double> leftTrigger, Supplier<Double> rightTrigger) {
        return this.run(() -> {
            this.rightMotor.setControl(this.controller.withOutput(Constants.Climber.MAX_OUTPUT.times(leftTrigger.get() - rightTrigger.get())));
        });
    }

    public Command off() {
        return this.run(() -> this.rightMotor.setControl(this.controller.withOutput(0.0)));
    }
}