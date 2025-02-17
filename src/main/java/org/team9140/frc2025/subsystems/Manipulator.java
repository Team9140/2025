package org.team9140.frc2025.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team9140.frc2025.Constants;

public class Manipulator extends SubsystemBase {
    private WPI_VictorSPX manipulatorMotor;
    private static Manipulator instance;

    public Manipulator() {
        this.manipulatorMotor = new WPI_VictorSPX(0);
    }

    public static Manipulator getInstance() {
        return (instance == null) ? instance = new Manipulator() : instance;
    }

    public Command turnOff(){
        return this.setVoltage(Constants.Manipulator.OFF);
    }

    public Command intake(){
        return this.setVoltage(
                Constants.Manipulator.INTAKE_VOLTAGE
        ).andThen(
                this.setVoltage(Constants.Manipulator.HOLD_VOLTAGE)
        );
    }

    public Command outtake(){
        return this.setVoltage(Constants.Manipulator.OUTTAKE_VOLTAGE)
                .andThen(
                    this.turnOff()
                );
    }

    public Command setVoltage(double voltage) {
        return run(() -> this.manipulatorMotor.setVoltage(voltage));
    }
}
