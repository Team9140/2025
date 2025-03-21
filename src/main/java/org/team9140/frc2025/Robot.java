// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team9140.frc2025;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static edu.wpi.first.units.Units.Seconds;

public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private final RobotContainer robotContainer;

    public Robot() {
        super(Constants.LOOP_PERIOD.in(Seconds));
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        SignalLogger.setPath("/media/sda/logs");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.periodic();
    }

    @Override
    public void disabledInit() {
        
    }

    @Override
    public void disabledPeriodic() {
        if (autonomousCommand == null && DriverStation.getAlliance().isPresent())
            autonomousCommand = robotContainer.getAutonomousCommand();
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        if (autonomousCommand == null)
            autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }

        SignalLogger.start();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        SignalLogger.start();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
        SignalLogger.stop();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
