// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team9140.frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team9140.frc2025.generated.TunerConstants;
import org.team9140.frc2025.subsystems.*;
import org.team9140.lib.MazeRunner;

import static edu.wpi.first.units.Units.*;


public class RobotContainer
{
    private Command autonomousCommand = Commands.print("No autonomous sequence has been set.");

    private MazeRunner path;

    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.getDrivetrain();
    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    private final Elevator elevator = Elevator.getInstance();
    private final Manipulator manipulator = Manipulator.getInstance();
    private final Funnel funnel = Funnel.getInstance();
    private final Canndle candle = Canndle.getInstance();

    private final LimeLight limeA = new LimeLight("limelight-a", this.drivetrain::acceptVisionMeasurement);
    private final LimeLight limeB = new LimeLight("limelight-b", this.drivetrain::acceptVisionMeasurement);
    private final LimeLight limeC = new LimeLight("limelight-c", this.drivetrain::acceptVisionMeasurement);

    Trigger enabledTrigger = new Trigger(DriverStation::isEnabled);
    Trigger connectedTrigger = new Trigger(DriverStation::isDSAttached);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        this.candle.setDefaultCommand(this.candle.solidAllianceColor());

        this.drivetrain
                .setDefaultCommand(drivetrain.teleopDrive(controller::getLeftX, controller::getLeftY, controller::getRightX));

        this.controller.rightTrigger().whileTrue(this.manipulator.outtakeCoral());

        this.controller.rightBumper().whileTrue(this.manipulator.intakeCoral().alongWith(this.funnel.intakeCoral()).withName("intake coral"));
        this.controller.leftBumper().whileTrue(this.manipulator.reverse().alongWith(this.funnel.reverse()).withName("unstick coral"));

        this.controller.povLeft().onTrue(new PrintCommand("snap L"));
        this.controller.povRight().onTrue(new PrintCommand("snap R"));
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

        limeA.start();
        limeB.start();
        limeC.start();

        enabledTrigger.onTrue(Commands.runOnce(() -> {
            System.out.println("enabling");
            limeA.setIMUMode(2);
            limeB.setIMUMode(2);
            limeC.setIMUMode(2);
        })).onFalse(Commands.runOnce(() -> {
            System.out.println("disabling");
            limeA.setIMUMode(1);
            limeB.setIMUMode(1);
            limeC.setIMUMode(1);
        }));

        connectedTrigger.onTrue(this.candle.blinkColorEndsAlliance(Canndle.GREEN, 0.1, 1.0));
    }

    public void periodic() {
        limeA.setRobotOrientation(this.drivetrain.getState().Pose.getRotation());
        limeB.setRobotOrientation(this.drivetrain.getState().Pose.getRotation());
        limeC.setRobotOrientation(this.drivetrain.getState().Pose.getRotation());
    }

    public Command getAutonomousCommand() {
        return autonomousCommand;
    }
}
