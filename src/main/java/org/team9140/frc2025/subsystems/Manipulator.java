package org.team9140.frc2025.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliseconds;
import static org.team9140.frc2025.Constants.Manipulator.HOLD_VOLTAGE_ALGAE;
import static org.team9140.frc2025.Constants.Manipulator.INTAKE_VOLTAGE_ALGAE;
import static org.team9140.frc2025.Constants.Manipulator.INTAKE_VOLTAGE_CORAL;
import static org.team9140.frc2025.Constants.Manipulator.MANIPULATOR_CONTINUOUS_CURRENT_LIMIT;
import static org.team9140.frc2025.Constants.Manipulator.MANIPULATOR_PEAK_CURRENT_DURATION;
import static org.team9140.frc2025.Constants.Manipulator.MANIPULATOR_PEAK_CURRENT_LIMIT;
import static org.team9140.frc2025.Constants.Manipulator.OUTTAKE_VOLTAGE_ALGAE;
import static org.team9140.frc2025.Constants.Manipulator.OUTTAKE_VOLTAGE_CORAL;

import org.team9140.frc2025.Constants.Ports;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
    private final TalonSRX manipulatorMotor;

    private Manipulator() {
        this.manipulatorMotor = new TalonSRX(Ports.MANIPULATOR_MOTOR);
        this.manipulatorMotor.setInverted(false);
        this.manipulatorMotor.setNeutralMode(NeutralMode.Brake);

        this.manipulatorMotor.configPeakCurrentLimit((int) MANIPULATOR_PEAK_CURRENT_LIMIT.in(Amps));
        this.manipulatorMotor.configPeakCurrentDuration((int) MANIPULATOR_PEAK_CURRENT_DURATION.in(Milliseconds));
        this.manipulatorMotor.configContinuousCurrentLimit((int) MANIPULATOR_CONTINUOUS_CURRENT_LIMIT.in(Amps));
        this.manipulatorMotor.enableCurrentLimit(true);

        this.setDefaultCommand(this.turnOff());
    }

    private static Manipulator instance;

    public static Manipulator getInstance() {
        return (instance == null) ? instance = new Manipulator() : instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("manipulator voltage", this.manipulatorMotor.getMotorOutputVoltage());
        SmartDashboard.putNumber("manipulator current", this.manipulatorMotor.getStatorCurrent());
        SmartDashboard.putString("manipulator command",
                this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "null");
    }

    public Command turnOff() {
        return this.setVoltage(0).withName("off");
    }

    public Command intakeCoral() {
        return this.setVoltage(INTAKE_VOLTAGE_CORAL).withName("intake coral");
    }

    public Command holdAlgae() {
        return this.setVoltage(HOLD_VOLTAGE_ALGAE).withName("hold algae");
    }

    public Command intakeAlgae() {
        return this.setVoltage(INTAKE_VOLTAGE_ALGAE).withName("intake algae");
    }

    public Command outtakeCoral() {
        return this.setVoltage(OUTTAKE_VOLTAGE_CORAL).withName("throw coral");
    }

    public Command outtakeAlgae() {
        return this.setVoltage(OUTTAKE_VOLTAGE_ALGAE).withName("throw algae");
    }

    public Command setVoltage(double voltage) {
        return run(() -> this.manipulatorMotor.set(TalonSRXControlMode.PercentOutput, voltage / 12.0));
    }

    public Command reverse() {
        return this.setVoltage(-INTAKE_VOLTAGE_CORAL).withName("unstick coral");
    }
}
