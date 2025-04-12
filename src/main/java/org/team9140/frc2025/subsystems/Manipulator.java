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

import com.ctre.phoenix6.SignalLogger;
import org.team9140.frc2025.Constants;
import org.team9140.frc2025.Constants.Ports;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Manipulator extends SubsystemBase {
    private enum Holdables {
        WATER,
        ALGAE,
        CORAL
    }

    private final TalonSRX manipulatorMotor = new TalonSRX(Ports.MANIPULATOR_MOTOR);;
    private Holdables currentItem = Holdables.WATER;

    private Manipulator() {
        this.manipulatorMotor.setInverted(false);
        this.manipulatorMotor.setNeutralMode(NeutralMode.Brake);

        this.manipulatorMotor.configPeakCurrentLimit((int) MANIPULATOR_PEAK_CURRENT_LIMIT.in(Amps));
        this.manipulatorMotor.configPeakCurrentDuration((int) MANIPULATOR_PEAK_CURRENT_DURATION.in(Milliseconds));
        this.manipulatorMotor.configContinuousCurrentLimit((int) MANIPULATOR_CONTINUOUS_CURRENT_LIMIT.in(Amps));
        this.manipulatorMotor.enableCurrentLimit(true);

        this.manipulatorMotor.set(TalonSRXControlMode.PercentOutput, 0);

        this.setDefaultCommand(this.off());
    }

    private static Manipulator instance;

    public static Manipulator getInstance() {
        return (instance == null) ? instance = new Manipulator() : instance;
    }

    @Override
    public void periodic() {
        Command curr = this.getCurrentCommand();
        SignalLogger.writeString("manipulator command", curr != null ? curr.getName() : "N/A");

//        SmartDashboard.putNumber("manipulator voltage", this.manipulatorMotor.getMotorOutputVoltage());
//        SmartDashboard.putNumber("manipulator current", this.manipulatorMotor.getStatorCurrent());
    }

//    public final Trigger hasCoral = new Trigger(() -> this.currentItem.equals(Holdables.CORAL));
    public final Trigger hasCoral = new Trigger(() -> Amps.of(this.manipulatorMotor.getStatorCurrent()).gt(Constants.Manipulator.HOLD_AMPERAGE_ALGAE));
    public final Trigger hasAlgae = new Trigger(() -> this.currentItem.equals(Holdables.ALGAE));

    public Command off() {
        return this.run(() -> {
            switch (currentItem) {
                case CORAL:
                case WATER:
                    this.manipulatorMotor.set(TalonSRXControlMode.PercentOutput, 0);
                    break;
                case ALGAE:
                    this.manipulatorMotor.set(TalonSRXControlMode.PercentOutput, HOLD_VOLTAGE_ALGAE / 12.0);
                    break;
            }
        }).withName("off");
    }

    public Command intakeCoral() {
        return this.runOnce(() -> this.currentItem = Holdables.CORAL)
                .andThen(this.setVoltage(INTAKE_VOLTAGE_CORAL).withName("intake coral"));
    }

    public Command intakeAlgae() {
        return this.runOnce(() -> this.currentItem = Holdables.ALGAE)
                .andThen(this.setVoltage(INTAKE_VOLTAGE_ALGAE).withName("intake algae"));
    }

    public Command outtakeCoral() {
        return this.setVoltage(OUTTAKE_VOLTAGE_CORAL).withName("throw coral");
    }

    public Command outtakeAlgae() {
        return this.runOnce(() -> this.currentItem = Holdables.WATER)
                .andThen(this.setVoltage(OUTTAKE_VOLTAGE_ALGAE).withName("throw algae"));
    }

    public Command setVoltage(double voltage) {
        return run(() -> this.manipulatorMotor.set(TalonSRXControlMode.PercentOutput, voltage / 12.0));
    }

    public Command reverse() {
        return this.setVoltage(-INTAKE_VOLTAGE_CORAL).withName("unstick coral");
    }
}
