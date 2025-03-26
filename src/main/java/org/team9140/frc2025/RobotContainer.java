// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team9140.frc2025;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.HashMap;
import java.util.Optional;

import org.team9140.frc2025.Constants.ElevatorSetbacks;
import org.team9140.frc2025.generated.TunerConstants;
import org.team9140.frc2025.helpers.LimelightHelpers;
import org.team9140.frc2025.subsystems.Canndle;
import org.team9140.frc2025.subsystems.Climber;
import org.team9140.frc2025.subsystems.CommandSwerveDrivetrain;
import org.team9140.frc2025.subsystems.Elevator;
import org.team9140.frc2025.subsystems.Funnel;
import org.team9140.frc2025.subsystems.LimeLight;
import org.team9140.frc2025.subsystems.Manipulator;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.getDrivetrain();
    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    private final Elevator elevator = Elevator.getInstance();
    private final Manipulator manipulator = Manipulator.getInstance();
    private final Funnel funnel = Funnel.getInstance();
    private final Canndle candle = Canndle.getInstance();
    private final Climber climber = Climber.getInstance();

    private final LimeLight limeA = new LimeLight("limelight-a", this.drivetrain::acceptVisionMeasurement);
    private final LimeLight limeB = new LimeLight("limelight-b", this.drivetrain::acceptVisionMeasurement);
    // private final LimeLight limeC = new LimeLight("limelight-c",
    // this.drivetrain::acceptVisionMeasurement);

    Trigger enabledTrigger = new Trigger(DriverStation::isEnabled);
    Trigger connectedTrigger = new Trigger(DriverStation::isDSAttached);

    public RobotContainer() {
        limeA.setIMUMode(1);
        limeB.setIMUMode(1);

        configureBindings();
        loadTrajectories();
    }

    private boolean stickInput() {
        return Math.abs(this.controller.getLeftX()) > 0.35
                || Math.abs(this.controller.getLeftY()) > 0.35
                || Math.abs(this.controller.getRightX()) > 0.35;
    }

    private final Trigger exitAutoAlign = new Trigger(this::stickInput);

    private void configureBindings() {
        this.candle.setDefaultCommand(this.candle.solidAllianceColor());
        this.climber.setDefaultCommand(this.climber.off());

        drivetrain
                .setDefaultCommand(
                        drivetrain.teleopDrive(controller::getLeftX, controller::getLeftY,
                                controller::getRightX));

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

        this.controller.y().and(this.controller.povCenter())
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

        controller.start().onTrue(this.drivetrain.resetGyroCommand());
        controller.back().whileTrue(this.drivetrain.goToPose(() -> new Pose2d(1, 0, new Rotation2d())));

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

        // limeA.start();
        limeB.start();
        // limeC.start();

        // connectedTrigger.onTrue(
        //         this.candle.blinkColorEndsAlliance(Canndle.GREEN, Seconds.of(0.1), Seconds.of(2.0)));
    }

    public void periodic() {
        if (DriverStation.isEnabled()) {
        //     limeA.setIMUMode(2);
            limeB.setIMUMode(2);
        } else {
        //     limeA.setIMUMode(1);
            limeB.setIMUMode(1);
        }

        // limeA.setRobotOrientation(this.drivetrain.getState().Pose.getRotation());
        limeB.setRobotOrientation(this.drivetrain.getState().Pose.getRotation());
        // limeC.setRobotOrientation(this.drivetrain.getState().Pose.getRotation());

        // Right Side
        // LimelightHelpers.setCameraPose_RobotSpace("limelight-b",
        //         Units.inchesToMeters(7.794), // Forward+
        //         Units.inchesToMeters(10.347), // Right+??
        //         Units.inchesToMeters(9.637), // Up+
        //         0, 0, 15.0);

        // Left Side
        // LimelightHelpers.setCameraPose_RobotSpace("limelight-c",
        // Units.inchesToMeters(7.794),
        // Units.inchesToMeters(10.347),
        // Units.inchesToMeters(9.637), 0, 0, -15.0);

        // Top Camera
        // LimelightHelpers.setCameraPose_RobotSpace("limelight-a",
        //         Units.inchesToMeters(1.739), // Forward+
        //         Units.inchesToMeters(0), // Right+???
        //         Units.inchesToMeters(34.677), // Up+
        //         0, 10.118, -180.0);

        // only use reef tags for pose on low camera
        // LimelightHelpers.SetFiducialIDFiltersOverride("limelight-b",
        //         new int[] { 17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11 });

        // // only use coral station tags for pose on back camera
        // LimelightHelpers.SetFiducialIDFiltersOverride("limelight-a", new int[] { 1, 2, 12, 13 });
    }

    HashMap<String, Trajectory<SwerveSample>> trajects;

    public void loadTrajectories() {
        trajects = new HashMap<>();

        String[] trajNames = Choreo.availableTrajectories();

        for (String nm : trajNames) {
            Optional<Trajectory<SwerveSample>> optTraj = Choreo.loadTrajectory(nm);
            if (optTraj.isPresent()) {
                trajects.put(nm, optTraj.get());
            } else {
                System.out.println("traject " + nm + " is null");
                trajects.put(nm, null);
            }
        }
    }

    public Command getAutonomousCommand() {
        return rightTwoCoral();
    }

    private Command leftTwoCoralDriveOnly() {
        return new SequentialCommandGroup(
                drivetrain.follow(trajects.get("left_start_to_J")),
                new WaitCommand(Seconds.of(1.0)),
                drivetrain.follow(trajects.get("J_to_left_feed")),
                new WaitCommand(Seconds.of(1.0)),
                drivetrain.follow(trajects.get("left_feed_to_L"))
        );
    }

    private Command leftTwoCoral() {
        return new SequentialCommandGroup(
                drivetrain.follow(trajects.get("left_start_to_J")),
                new WaitCommand(Seconds.of(0.25)),
                elevator.moveToPosition(Constants.Elevator.L4_coral_height),
                new WaitCommand(Seconds.of(0.25)),
                manipulator.outtakeCoral().withTimeout(Seconds.of(1.0)),
                elevator.moveToPosition(Constants.Elevator.STOW_height),
                manipulator.intakeCoral()
                        .alongWith(funnel.intakeCoral())
                        .withDeadline(
                                drivetrain.follow(trajects.get("J_to_left_feed"))
                                        .andThen(new WaitCommand(Seconds.of(0.5)))
                                        .andThen(drivetrain.follow(trajects.get("left_feed_to_L")))),
                manipulator.off().withTimeout(Seconds.of(0.001)),
                new WaitCommand(Seconds.of(0.5)),
                elevator.moveToPosition(Constants.Elevator.L4_coral_height),
                new WaitCommand(Seconds.of(0.25)),
                manipulator.outtakeCoral().withTimeout(Seconds.of(1.0)),
                elevator.moveToPosition(Constants.Elevator.STOW_height),
                new PrintCommand("lmao"));
    }

    private Command rightTwoCoral() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> trajects.get("right_start_to_F").sampleAt(0, false)).withTimeout(Seconds.of(2.0)),
                drivetrain.follow(trajects.get("right_start_to_F")),
                new WaitCommand(Seconds.of(0.25)),
                drivetrain.coralReefDrive(ElevatorSetbacks.L4, true).withTimeout(Seconds.of(1.0)),
                drivetrain.stop(),
                new WaitCommand(Seconds.of(0.25)),
                elevator.moveToPosition(Constants.Elevator.L4_coral_height),
                new WaitCommand(Seconds.of(0.25)),
                manipulator.outtakeCoral().withTimeout(Seconds.of(1.0)),
                elevator.moveToPosition(Constants.Elevator.STOW_height),
                manipulator.intakeCoral()
                        .alongWith(funnel.intakeCoral())
                        .withDeadline(
                                drivetrain.follow(trajects.get("F_to_right_feed"))
                                        .andThen(new WaitCommand(Seconds.of(0.5)))
                                        .andThen(drivetrain.follow(trajects.get("right_feed_to_D")))),
                manipulator.off().withTimeout(Seconds.of(0.001)),
                new WaitCommand(Seconds.of(0.5)),
                elevator.moveToPosition(Constants.Elevator.L4_coral_height),
                new WaitCommand(Seconds.of(0.25)),
                manipulator.outtakeCoral().withTimeout(Seconds.of(1.0)),
                elevator.moveToPosition(Constants.Elevator.STOW_height),
                new PrintCommand("lmao"));
    }

    private Command practiceFieldReefDriveBy() {
        return drivetrain.follow(trajects.get("practice_field"));
    }

    private Command squareDanceTwoCoralTest() {
        return new SequentialCommandGroup(
                drivetrain.follow(trajects.get("x0y0_to_x1y0")),
                new PrintCommand("finished x0y0 to x1y0"),
                new WaitCommand(Seconds.of(0.25)),
                elevator.moveToPosition(Constants.Elevator.L4_coral_height),
                new WaitCommand(Seconds.of(0.25)),
                manipulator.outtakeCoral().withTimeout(Seconds.of(1.0)),
                elevator.moveToPosition(Constants.Elevator.STOW_height),
                manipulator.intakeCoral()
                        .alongWith(funnel.intakeCoral())
                        .withDeadline(
                                drivetrain.follow(trajects.get("x1y0_to_x1y1"))
                                        .andThen(new PrintCommand("finished x1y0_to_x1y1"))
                                        .andThen(new WaitCommand(Seconds.of(0.5)))
                                        .andThen(drivetrain.follow(trajects.get("x1y1_to_x0y0")))
                                        .andThen(new PrintCommand("finished x1y1_to_x0y0"))),
                manipulator.off().withTimeout(Seconds.of(0.001)),
                new WaitCommand(Seconds.of(0.5)),
                elevator.moveToPosition(Constants.Elevator.L4_coral_height),
                new WaitCommand(Seconds.of(0.25)),
                manipulator.outtakeCoral().withTimeout(Seconds.of(1.0)),
                elevator.moveToPosition(Constants.Elevator.STOW_height),
                new PrintCommand("lmao"));
    }

}
