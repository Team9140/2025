package org.team9140.frc2025.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.Command;

public class Manipulator {
    WPI_VictorSPX manipulatorMotor = new WPI_VictorSPX(0);

    public Manipulator() {

    }

//    public Command turnOn(){
//
//    }

    public Command turnOff(){
        manipulatorMotor.setVoltage(0);
        return null;
    }

//    public Command moveBack(){
//
//    }

    public Command setVoltage(double voltage) {
        manipulatorMotor.setVoltage(voltage);
        return null;
    }
}
