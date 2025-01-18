// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team9140.frc2025;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import org.team9140.frc2025.generated.TunerConstants;
import org.team9140.frc2025.subsystems.CommandSwerveDrivetrain;


public class RobotContainer
{

    CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    CommandXboxController controller = new CommandXboxController(0);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        drivetrain
                .setDefaultCommand(drivetrain.teleopDrive(controller::getLeftX, controller::getLeftY, controller::getRightX));

        controller.start().onTrue(drivetrain.resetGyroCommand());

        controller.a().whileTrue(drivetrain.sysIdSteerD(Direction.kForward));
        controller.b().whileTrue(drivetrain.sysIdSteerD(Direction.kReverse));
        controller.x().whileTrue(drivetrain.sysIdSteerQ(Direction.kForward));
        controller.y().whileTrue(drivetrain.sysIdSteerQ(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
