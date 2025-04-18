package org.team9140.frc2025.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import org.team9140.frc2025.Constants;
import org.team9140.frc2025.Constants.Ports;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {

    private final TalonFX motor;
    private VoltageOut output;

    private Funnel() {
        this.motor = new TalonFX(Ports.FUNNEL_MOTOR, "sigma");
        this.output = new VoltageOut(0);

        this.motor.getConfigurator().apply(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));

        this.motor.getConfigurator().apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Funnel.STATOR_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Constants.Funnel.SUPPLY_LIMIT)
                .withSupplyCurrentLimitEnable(true));

        this.motor.setControl(this.output);

        this.setDefaultCommand(this.turnOff());
    }

    private static Funnel instance;

    public static Funnel getInstance() {
        return (instance == null) ? instance = new Funnel() : instance;
    }

    public void periodic() {
        Command curr = this.getCurrentCommand();
        SignalLogger.writeString("funnel command", curr != null ? curr.getName() : "N/A");

//        SmartDashboard.putNumber("funnel voltage", this.motor.getMotorVoltage(false).getValue().in(Volts));
//        SmartDashboard.putNumber("funnel current", this.motor.getStatorCurrent(false).getValue().in(Amps));
        SmartDashboard.putString("funnel command",
                this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "null");
    }

    public Command turnOff() {
        return this.runOnce(() -> this.motor.setControl(output.withOutput(Volts.of(0)))).withName("off");
    }

    public Command intakeCoral() {
        return this.setVoltage(Constants.Funnel.INTAKE_VOLTAGE).withName("intake coral");
    }

    public Command setVoltage(Voltage v) {
        return run(() -> this.motor.setControl(this.output.withOutput(v)));
    }

    public Command reverse() {
        return this.setVoltage(Constants.Funnel.UNSTICK_VOLTAGE).withName("unstick coral");
    }
}
