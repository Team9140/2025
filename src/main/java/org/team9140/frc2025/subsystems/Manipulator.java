package org.team9140.frc2025.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static org.team9140.frc2025.Constants.Manipulator.*;

public class Manipulator extends SubsystemBase {
    private final TalonSRX manipulatorMotor;
    private static Manipulator instance;
    private final CANrange canRange;
    private final TalonFX funnelMotor;
    private final VelocityVoltage funnelController;

    public Manipulator() {
        this.manipulatorMotor = new TalonSRX(MANIPULATOR_MOTOR_DEVICE_NUM);
        this.funnelMotor = new TalonFX(FUNNEL_MOTOR_DEVICE_NUM);
        this.canRange = new CANrange(CAN_RANGE_DEVICE_NUM);
        this.funnelController = new VelocityVoltage(0).withEnableFOC(true);

        this.manipulatorMotor.configPeakCurrentLimit(MANIPULATOR_PEAK_CURRENT_LIMIT);
        this.manipulatorMotor.configPeakCurrentDuration(MANIPULATOR_PEAK_CURRENT_DURATION);
        this.manipulatorMotor.configContinuousCurrentLimit(MANIPULATOR_CONTINUOUS_CURRENT_LIMIT);
        this.manipulatorMotor.enableCurrentLimit(true);

        CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();
        ProximityParamsConfigs proximity = new ProximityParamsConfigs();
        proximity.MinSignalStrengthForValidMeasurement = MIN_SIGNAL_STRENGTH;
        proximity.ProximityHysteresis = PROXIMITY_HYSTERESIS;
        proximity.ProximityThreshold = PROXIMITY_THRESHOLD;
        canRange.getConfigurator().apply(proximity);

        HardwareLimitSwitchConfigs limitsConfig = new HardwareLimitSwitchConfigs();
        limitsConfig.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANrange;
        limitsConfig.ReverseLimitSource = ReverseLimitSourceValue.Disabled;
        limitsConfig.withForwardLimitRemoteSensorID(canRange.getDeviceID());
        limitsConfig.ForwardLimitAutosetPositionEnable = true;
        limitsConfig.ForwardLimitAutosetPositionValue = FORWARD_AUTOSET;

        funnelMotor.getConfigurator().apply(limitsConfig);
        canRange.getConfigurator().apply(canRangeConfig);
    }

    public static Manipulator getInstance() {
        return (instance == null) ? instance = new Manipulator() : instance;
    }

    public Command stopFunnel() {
        return run(() -> funnelMotor.setControl(funnelController.withVelocity(0)));
    }

    public Command turnOff(){
        return this.setVoltage(0).alongWith(stopFunnel());
    }

    public Command startFunnel(){
        return run(()-> funnelMotor.setControl(funnelController.withVelocity(FUNNEL_CONTROLLER_VELOCITY)));
    }

    public Command intakeCoral(){
        return this.setVoltage(
                INTAKE_VOLTAGE_CORAL
        ).alongWith(startFunnel()).until(() -> canRange.getDistance(true).getValue().compareTo(CORAL_DISTANCE) <= 0).andThen(
                this.setVoltage(INTAKE_VOLTAGE_CORAL).raceWith(new WaitCommand(INTAKE_CORAL_TIME))
        ).andThen(
                this.turnOff()
        );
    }

    public Command holdAlgae(){
        return this.setVoltage(HOLD_VOLTAGE_ALGAE);
    }

    public Command intakeAlgae(){
        return this.setVoltage(INTAKE_VOLTAGE_ALGAE);
    }

    public Command outtakeCoral(){
        return this.setVoltage(OUTTAKE_VOLTAGE_CORAL);
    }

    public Command outtakeAlgae(){
        return this.setVoltage(OUTTAKE_VOLTAGE_ALGAE);
    }

    public Command setVoltage(double voltage) {
        return runOnce(() -> this.manipulatorMotor.set(TalonSRXControlMode.PercentOutput, voltage));
    }
}
