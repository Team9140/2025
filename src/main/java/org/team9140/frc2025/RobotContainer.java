// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team9140.frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import org.team9140.frc2025.generated.TunerConstants;
import org.team9140.frc2025.subsystems.CommandSwerveDrivetrain;
import org.team9140.lib.MazeRunner;

import static edu.wpi.first.units.Units.MetersPerSecond;


public class RobotContainer
{
    private Command autonomousCommand = Commands.print("No autonomous sequence has been set.");

    private MazeRunner path;

    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    CommandXboxController controller = new CommandXboxController(0);

    public RobotContainer() {
        this.path = new MazeRunner("funner", drivetrain, DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
        this.path.atEventTime("test1").onTrue(new PrintCommand("test1"));
        this.path.atEventTime("test2").onTrue(new PrintCommand("test2"));
        this.path.atTime(2.5).onTrue(new PrintCommand("2.5 seconds"));
        this.path.atPose(new Pose2d(4.631613731384277, 7.529511451721191, Rotation2d.fromDegrees(90)), 0.05, 0.03).onTrue(
                new PrintCommand("Pose 4")
        );
        this.path.atTranslation(new Translation2d(2.885009765625, 5.642167568206787), 0.1).onTrue(
                        new PrintCommand("Pose 5")
                );

        this.autonomousCommand = this.path.gimmeCommand();

        configureBindings();
    }

    private void configureBindings() {
        drivetrain
                .setDefaultCommand(drivetrain.teleopDrive(controller::getLeftX, controller::getLeftY, controller::getRightX));

//        controller.a().whileTrue(drivetrain.sysIdSteerD(Direction.kForward));
//        controller.b().whileTrue(drivetrain.sysIdSteerD(Direction.kReverse));
//        controller.x().whileTrue(drivetrain.sysIdSteerQ(Direction.kForward));
//        controller.y().whileTrue(drivetrain.sysIdSteerQ(Direction.kReverse));

//        controller.a().whileTrue(drivetrain.sysIdDriveD(Direction.kForward));
//        controller.b().whileTrue(drivetrain.sysIdDriveD(Direction.kReverse));
//        controller.x().whileTrue(drivetrain.sysIdDriveQ(Direction.kForward));
//        controller.y().whileTrue(drivetrain.sysIdDriveQ(Direction.kReverse));

//        controller.a().whileTrue(drivetrain.sysIdRotateD(Direction.kForward));
//        controller.b().whileTrue(drivetrain.sysIdRotateD(Direction.kReverse));
//        controller.x().whileTrue(drivetrain.sysIdRotateQ(Direction.kForward));
//        controller.y().whileTrue(drivetrain.sysIdRotateQ(Direction.kReverse));

        this.drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        this.drivetrain.resetPose(this.path.getInitialPose());
        return autonomousCommand;
    }
}
