// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team9140.frc2025;

import static edu.wpi.first.units.Units.Seconds;

import org.team9140.frc2025.commands.AutonomousRoutines;
import org.team9140.frc2025.generated.TunerConstantsComp;
import org.team9140.frc2025.helpers.AutoAiming;
import org.team9140.frc2025.subsystems.Canndle;
import org.team9140.frc2025.subsystems.Climber;
import org.team9140.frc2025.subsystems.CommandSwerveDrivetrain;
import org.team9140.frc2025.subsystems.Elevator;
import org.team9140.frc2025.subsystems.Funnel;
import org.team9140.frc2025.subsystems.LimeLight;
import org.team9140.frc2025.subsystems.Manipulator;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandSwerveDrivetrain drivetrain = TunerConstantsComp.getDrivetrain();
    private final SwerveTelemetry logger = new SwerveTelemetry();
    private final Elevator elevator = Elevator.getInstance();
    private final Manipulator manipulator = Manipulator.getInstance();
    private final Funnel funnel = Funnel.getInstance();
    private final Canndle candle = Canndle.getInstance();
    private final Climber climber = Climber.getInstance();

    private final LimeLight limeA = new LimeLight("limelight-a", this.drivetrain::acceptVisionMeasurement);
    private final LimeLight limeB = new LimeLight("limelight-b", this.drivetrain::acceptVisionMeasurement);
    private final LimeLight limeC = new LimeLight("limelight-c", this.drivetrain::acceptVisionMeasurement);

    Trigger enabledTrigger = new Trigger(DriverStation::isEnabled);
    Trigger connectedTrigger = new Trigger(DriverStation::isDSAttached);

    public RobotContainer() {
        limeA.setIMUMode(1);
        limeB.setIMUMode(1);
        limeC.setIMUMode(1);

        configureBindings();
    }

    private boolean stickInput() {
        return Math.abs(this.controller.getLeftX()) > 0.35
                || Math.abs(this.controller.getLeftY()) > 0.35
                || Math.abs(this.controller.getRightX()) > 0.35;
    }

    private final Trigger exitAutoAlign = new Trigger(this::stickInput);

    private void configureBindings() {
        drivetrain
                .setDefaultCommand(
                        drivetrain.teleopDrive(controller::getLeftX, controller::getLeftY,
                                controller::getRightX).ignoringDisable(true));

        controller.rightTrigger().and(manipulator.hasCoral)
                .onTrue(this.manipulator.outtakeCoral().until(this.controller.rightTrigger().negate()));
        controller.rightTrigger().and(manipulator.hasAlgae)
                .onTrue(this.manipulator.outtakeAlgae().until(this.controller.rightTrigger().negate()));

        controller.rightBumper().and(elevator.isStowed).whileTrue(
                this.manipulator.intakeCoral().alongWith(this.funnel.intakeCoral())
                        .withName("intake coral"));
        controller.leftBumper()
                .whileTrue(this.manipulator.reverse().alongWith(this.funnel.reverse())
                        .withName("unstick coral"));

        this.controller.rightBumper().and(elevator.isStowed.negate())
                .whileTrue(this.manipulator.intakeAlgae().withName("intake algae"));

        this.controller.y().and(this.controller.povRight())
                .onTrue(this.drivetrain.coralReefDrive(Constants.ElevatorSetbacks.L4, false)
                        .alongWith(this.elevator
                                .moveToPosition(Constants.Elevator.L4_coral_height))
                        .alongWith(this.candle.blinkColorForever(Canndle.PURPLE,
                                Seconds.of(0.5)))
                        .until(this::stickInput)
                        .withName("high coral R"));

        this.controller.y().and(this.controller.povLeft())
                .onTrue(this.drivetrain.coralReefDrive(Constants.ElevatorSetbacks.L4, true)
                        .alongWith(this.elevator
                                .moveToPosition(Constants.Elevator.L4_coral_height))
                        .alongWith(this.candle.blinkColorForever(Canndle.PURPLE,
                                Seconds.of(0.5)))
                        .until(this::stickInput)
                        .withName("high coral L"));

        this.controller.y().and(this.controller.povCenter()).and(this.manipulator.hasAlgae)
                .onTrue(this.elevator.moveToPosition(Constants.Elevator.NET_HEIGHT)
                        .withName("net height"));

        this.controller.b().and(this.controller.povRight())
                .onTrue(this.drivetrain.coralReefDrive(Constants.ElevatorSetbacks.L3, false)
                        .alongWith(this.elevator
                                .moveToPosition(Constants.Elevator.L3_coral_height))
                        .alongWith(this.candle.blinkColorForever(Canndle.PURPLE,
                                Seconds.of(0.5)))
                        .until(this::stickInput)
                        .withName("highish (level 3) coral R"));

        this.controller.b().and(this.controller.povLeft())
                .onTrue(this.drivetrain.coralReefDrive(Constants.ElevatorSetbacks.L3, true)
                        .alongWith(this.elevator
                                .moveToPosition(Constants.Elevator.L3_coral_height))
                        .alongWith(this.candle.blinkColorForever(Canndle.PURPLE,
                                Seconds.of(0.5)))
                        .until(this::stickInput)
                        .withName("highish (level 3) coral L"));

        this.controller.b().and(this.controller.povCenter())
                .onTrue(this.elevator
                        .moveToPosition(Constants.Elevator.L3_ALGAE_height)
                        .until(this::stickInput)
                        .withName("highish (level 3) algae center"));

        this.elevator.isAlgaeing.and(this.controller.rightBumper())
                .onTrue(this.drivetrain.algaeReefDrive()
                        .alongWith(this.candle.blinkColorForever(Canndle.PURPLE,
                                Seconds.of(0.5)))
                        .alongWith(this.elevator
                                .moveToPosition(() -> AutoAiming.getClosestFace(
                                        this.drivetrain.getState().Pose
                                                .getTranslation())
                                        .getAlgaeElevatorHeight()))
                        .until(this::stickInput)
                        .withName("autoaiming algae"));

        this.controller.a().and(this.controller.povRight())
                .onTrue(this.drivetrain.coralReefDrive(Constants.ElevatorSetbacks.L2, false)
                        .alongWith(this.elevator
                                .moveToPosition(Constants.Elevator.L2_coral_height))
                        .alongWith(this.candle.blinkColorForever(Canndle.PURPLE,
                                Seconds.of(0.5)))
                        .until(this::stickInput)
                        .withName("mid coral R"));

        this.controller.a().and(this.controller.povLeft())
                .onTrue(this.drivetrain.coralReefDrive(Constants.ElevatorSetbacks.L2, true)
                        .alongWith(this.elevator
                                .moveToPosition(Constants.Elevator.L2_coral_height))
                        .alongWith(this.candle.blinkColorForever(Canndle.PURPLE,
                                Seconds.of(0.5)))
                        .until(this::stickInput)
                        .withName("mid coral L"));

        this.controller.a().and(this.controller.povCenter())
                .onTrue(this.elevator
                        .moveToPosition(Constants.Elevator.L2_ALGAE_height)
                        .until(this::stickInput)
                        .withName("mid (level 2) algae center"));

        this.controller.x()
                .onTrue(this.elevator.moveToPosition(Constants.Elevator.STOW_height));

        this.controller.x()
                .whileTrue(this.climber.climb(this.controller.getHID()::getLeftTriggerAxis,
                        this.controller.getHID()::getRightTriggerAxis));

        // controller.start().onTrue(this.drivetrain.resetGyroCommand(Degrees.of(0)));
        // Pose2d target = new Pose2d(2, 4, new Rotation2d());
        // controller.back().whileTrue(this.drivetrain.goToPose(() -> target));
        this.controller.back().onTrue(this.climber.prep());
        this.elevator.isUp.onTrue(this.drivetrain.engageSlowMode())
                .onFalse(this.drivetrain.disengageSlowMode());

        this.exitAutoAlign.onTrue(this.candle.solidAllianceColor());

        // this.drivetrain.reachedPose
        // .onTrue(this.candle.blinkColorEndsOff(Canndle.GREEN, Seconds.of(0.1),
        // Seconds.of(2.0)));

        // controller.a().whileTrue(drivetrain.sysIdSteerD(Direction.kForward));
        // controller.b().whileTrue(drivetrain.sysIdSteerD(Direction.kReverse));
        // controller.x().whileTrue(drivetrain.sysIdSteerQ(Direction.kForward));
        // controller.y().whileTrue(drivetrain.sysIdSteerQ(Direction.kReverse));

        // controller.a().whileTrue(drivetrain.sysIdDriveD(Direction.kForward));
        // controller.b().whileTrue(drivetrain.sysIdDriveD(Direction.kReverse));
        // controller.x().whileTrue(drivetrain.sysIdDriveQ(Direction.kForward));
        // controller.y().whileTrue(drivetrain.sysIdDriveQ(Direction.kReverse));

        // controller.a().whileTrue(drivetrain.sysIdRotateD(Direction.kForward));
        // controller.b().whileTrue(drivetrain.sysIdRotateD(Direction.kReverse));
        // controller.x().whileTrue(drivetrain.sysIdRotateQ(Direction.kForward));
        // controller.y().whileTrue(drivetrain.sysIdRotateQ(Direction.kReverse));

        this.drivetrain.registerTelemetry(logger::telemeterize);

        limeA.start();
        limeB.start();
        limeC.start();

        // connectedTrigger.onTrue(
        //         this.candle.blinkColorEndsAlliance(Canndle.GREEN, Seconds.of(0.1), Seconds.of(2.0))
        //                 .ignoringDisable(true));
    }

    double prevTime = Utils.getCurrentTimeSeconds();

    public void periodic() {
        SmartDashboard.putNumber("period", -(prevTime - (prevTime = Utils.getCurrentTimeSeconds())));
        // if (DriverStation.isEnabled()) {
        // limeA.setIMUMode(2);
        // limeB.setIMUMode(2);
        // limeC.setIMUMode(2);
        // } else {
        // limeA.setIMUMode(1);
        // limeB.setIMUMode(1);
        // limeC.setIMUMode(1);
        // }

        // final Rotation2d robotAngle = this.drivetrain.getState().Pose.getRotation();
        // limeA.setRobotOrientation(robotAngle);
        // limeB.setRobotOrientation(robotAngle);
        // limeC.setRobotOrientation(robotAngle);
    }

    public Command getAutonomousCommand() {
        AutonomousRoutines routines = new AutonomousRoutines(this.drivetrain);
        return routines.threeCoralInsideLeft();
    }
}