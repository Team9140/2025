package org.team9140.frc2025.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team9140.frc2025.Constants;

import static edu.wpi.first.units.Units.*;

public class Manipulator extends SubsystemBase {
    private WPI_VictorSPX manipulatorMotor;
    private static Manipulator instance;
    CANrange canRange;
    TalonFX manipulatorTalon;

    public Manipulator() {
        this.manipulatorMotor = new WPI_VictorSPX(0);
        this.manipulatorTalon = new TalonFX(52);
        this.canRange = new CANrange(0);

        var canRangeConfig = new CANrangeConfiguration();
        var proximity = new ProximityParamsConfigs();
        proximity.MinSignalStrengthForValidMeasurement = 5000;
        proximity.ProximityHysteresis = 0.05;
        proximity.ProximityThreshold = 0.4;
        canRange.getConfigurator().apply(proximity);

        var limitsConfig = new HardwareLimitSwitchConfigs();
        limitsConfig.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANrange;
        limitsConfig.ReverseLimitSource = ReverseLimitSourceValue.Disabled;
        limitsConfig.withForwardLimitRemoteSensorID(canRange.getDeviceID());
        limitsConfig.ForwardLimitAutosetPositionEnable = true;
        limitsConfig.ForwardLimitAutosetPositionValue = 0.0;


        manipulatorTalon.getConfigurator().apply(limitsConfig);
        canRange.getConfigurator().apply(canRangeConfig);

    }

    public static Manipulator getInstance() {
        return (instance == null) ? instance = new Manipulator() : instance;
    }

    public Command turnOff(){
        return this.setVoltage(Constants.Manipulator.OFF);
    }

    public Command intakeCoral(){
        return this.setVoltage(
                Constants.Manipulator.INTAKE_VOLTAGE_CORAL
        ).until(() -> canRange.getDistance(true).getValue().compareTo(Constants.Manipulator.CORAL_DISTANCE) < 0).andThen(
                this.setVoltage(Constants.Manipulator.INTAKE_VOLTAGE_CORAL).raceWith(new WaitCommand(Constants.Manipulator.INTAKE_CORAL_TIME))
        ).andThen(
                this.turnOff()
        );
    }

    public Command intakeAlgae(){
        return this.setVoltage(
                Constants.Manipulator.INTAKE_VOLTAGE_ALGAE
        ).andThen(
                this.setVoltage(Constants.Manipulator.HOLD_VOLTAGE_ALGAE)
        );
    }

    public Command outtakeCoral(){
        return this.setVoltage(Constants.Manipulator.OUTTAKE_VOLTAGE_CORAL)
                .andThen(
                    this.turnOff()
                );
    }

    public Command outtakeAlgae(){
        return this.setVoltage(Constants.Manipulator.OUTTAKE_VOLTAGE_ALGAE)
                .andThen(
                        this.turnOff()
                );
    }

    public Command setVoltage(double voltage) {
        return run(() -> this.manipulatorMotor.setVoltage(voltage));
    }
}
