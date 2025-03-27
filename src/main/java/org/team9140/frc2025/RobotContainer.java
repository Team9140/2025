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
import org.team9140.frc2025.subsystems.Canndle;
import org.team9140.frc2025.subsystems.Limelight;
import org.team9140.frc2025.subsystems.TheMatrix;
import org.team9140.lib.MazeRunner;

import static edu.wpi.first.units.Units.*;


public class RobotContainer
{
    private Command autonomousCommand = Commands.print("No autonomous sequence has been set.");

    private MazeRunner path;

    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    CommandSwerveDrivetrain drivetrain = TunerConstants.getDrivetrain();

    CommandXboxController controller = new CommandXboxController(0);

    Canndle candle = Canndle.getInstance();

    Limelight b = new Limelight("limelight-b", drivetrain::addVisionMeasurement, null, null);

    TheMatrix matrix = new TheMatrix();


    public RobotContainer() {
        this.path = new MazeRunner("realgamestuff", drivetrain, DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
        this.path.atEventTime("First_Coral").onTrue((new PrintCommand("First_Coral")).alongWith(candle.flashColor(Canndle.ORANGE, 0.1)));
        this.path.atEventTime("Restock").onTrue((new PrintCommand("Restock")).alongWith(candle.flashColor(Canndle.BLUE, 0.1)));
        this.path.atEventTime("Second_Coral").onTrue((new PrintCommand("Second_Coral")).alongWith(candle.flashColor(Canndle.GREEN, 0.1)));
        this.path.atTime(2.5).onTrue(new PrintCommand("2.5 seconds"));
        this.path.atEventTime("Second_Restock").onTrue(new PrintCommand("Second_Restock").alongWith(candle.flashColor(Canndle.GREEN, 0.1)));
        this.path.atEventTime("Third_Coral").onTrue((new PrintCommand("Third_Coral")).alongWith(candle.flashColor(Canndle.RED, 0.1)));
        this.path.atEventTime("Stop").onTrue(new PrintCommand("Stop").alongWith(candle.flashColor(Canndle.BLUE, 0.1)));

        b.start();
        this.matrix.setupVision(drivetrain);

//        this.path.atEventTime("End_Test").onTrue(
//                new PrintCommand("End").alongWith(candle.flashColor(Canndle.PINK, 0.1))
//        );

        this.path.atPose(new Pose2d(1.249948263168335, 4.545039176940918, new Rotation2d(0)), 0.1, Degrees.of(5).in(Radians)).onTrue(
                (new PrintCommand("atPose test")).alongWith(candle.flashColor(Canndle.PINK, 0.1))
        );

//        this.path.atPose(new Pose2d(3.7773959636688232, 5.237298011779785, new Rotation2d(-1.016488417575178)), 5, Degrees.of(360).in(Radians)).onTrue(
//                (new PrintCommand("POSE2D_TEST")).alongWith(candle.flashColor(Canndle.PINK, 0.1))
//        );



        this.autonomousCommand = this.path.gimmeCommand();

        configureBindings();
    }

    private void configureBindings() {
        drivetrain
                .setDefaultCommand(drivetrain.teleopDrive(controller::getLeftX, controller::getLeftY, controller::getRightX));

        controller.start().onTrue(drivetrain.resetGyroCommand());

//        controller.a().whileTrue(drivetrain.sysIdSteerD(Direction.kForward));
//        controller.b().whileTrue(drivetrain.sysIdSteerD(Direction.kReverse));
//        controller.x().whileTrue(drivetrain.sysIdSteerQ(Direction.kForward));
//        controller.y().whileTrue(drivetrain.sysIdSteerQ(Direction.kReverse));
//
//        controller.a().whileTrue(drivetrain.sysIdDriveD(Direction.kForward));
//        controller.b().whileTrue(drivetrain.sysIdDriveD(Direction.kReverse));
//        controller.x().whileTrue(drivetrain.sysIdDriveQ(Direction.kForward));
//        controller.y().whileTrue(drivetrain.sysIdDriveQ(Direction.kReverse));

        controller.a().whileTrue(drivetrain.sysIdRotateD(Direction.kForward));
        controller.b().whileTrue(drivetrain.sysIdRotateD(Direction.kReverse));
        controller.x().whileTrue(drivetrain.sysIdRotateQ(Direction.kForward));
        controller.y().whileTrue(drivetrain.sysIdRotateQ(Direction.kReverse));

        this.drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autonomousCommand;
    }
}
