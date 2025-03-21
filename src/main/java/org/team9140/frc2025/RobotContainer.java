// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team9140.frc2025;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import org.team9140.frc2025.commands.AutonomousRoutines;
import org.team9140.frc2025.generated.TunerConstants;
import org.team9140.frc2025.helpers.LimelightHelpers;
import org.team9140.frc2025.subsystems.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
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
        // this.path = new MazeRunner("funner", drivetrain,
        // DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
        // this.path.atEventTime("test1").onTrue(new PrintCommand("test1"));
        // this.path.atEventTime("test2").onTrue(new PrintCommand("test2"));
        // this.path = new MazeRunner("themaze", drivetrain,
        // DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
        // this.path.atEventTime("First_Coral").onTrue((new
        // PrintCommand("First_Coral")).alongWith(candle.flashColor(Canndle.ORANGE,
        // 0.1)));
        // this.path.atEventTime("Restock").onTrue((new
        // PrintCommand("Restock")).alongWith(candle.flashColor(Canndle.BLUE, 0.1)));
        // this.path.atEventTime("Second_Coral").onTrue((new
        // PrintCommand("Second_Coral")).alongWith(candle.flashColor(Canndle.GREEN,
        // 0.1)));
        // this.path.atTime(2.5).onTrue(new PrintCommand("2.5 seconds"));
        // this.path.atEventTime("Second_Restock").onTrue(new
        // PrintCommand("Second_Restock").alongWith(candle.flashColor(Canndle.GREEN,
        // 0.1)));
        // this.path.atEventTime("Third_Coral").onTrue((new
        // PrintCommand("Third_Coral")).alongWith(candle.flashColor(Canndle.RED, 0.1)));
        // this.path.atEventTime("Stop").onTrue(new
        // PrintCommand("Stop").alongWith(candle.flashColor(Canndle.BLUE, 0.1)));

        // this.path.atEventTime("End_Test").onTrue(
        // new PrintCommand("End").alongWith(candle.flashColor(Canndle.PINK, 0.1))
        // );

        // this.path.atPose(new Pose2d(1.249948263168335, 4.545039176940918, new
        // Rotation2d(0)), 0.1, Degrees.of(5).in(Radians)).onTrue(
        // (new PrintCommand("atPose test")).alongWith(candle.flashColor(Canndle.PINK,
        // 0.1))
        // );

        // this.path.atPose(new Pose2d(3.7773959636688232, 5.237298011779785, new
        // Rotation2d(-1.016488417575178)), 5, Degrees.of(360).in(Radians)).onTrue(
        // (new PrintCommand("POSE2D_TEST")).alongWith(candle.flashColor(Canndle.PINK,
        // 0.1))
        // );

        // this.autonomousCommand = this.path.gimmeCommand();

        limeA.setIMUMode(1);
        limeB.setIMUMode(1);

        configureBindings();
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

        controller.rightTrigger().and(manipulator.hasCoral).onTrue(this.manipulator.outtakeCoral().until(this.controller.rightTrigger().negate()));
        controller.rightTrigger().and(manipulator.hasAlgae).onTrue(this.manipulator.outtakeAlgae().until(this.controller.rightTrigger().negate()));

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

        this.elevator.isUp.onTrue(this.drivetrain.engageSlowMode())
                .onFalse(this.drivetrain.disengageSlowMode());

        this.exitAutoAlign.onTrue(this.candle.solidAllianceColor());

        this.drivetrain.reachedPose
                .onTrue(this.candle.blinkColorEndsOff(Canndle.GREEN, Seconds.of(0.1), Seconds.of(2.0)));

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

        // enabledTrigger.onTrue(Commands.runOnce(() -> {
        // System.out.println("enabling");
        // limeA.setIMUMode(2);
        // limeB.setIMUMode(2);
        // // limeC.setIMUMode(2);
        // })).onFalse(Commands.runOnce(() -> {
        // System.out.println("disabling");
        // limeA.setIMUMode(1);
        // limeB.setIMUMode(1);
        // // limeC.setIMUMode(1);
        // }));

        connectedTrigger.onTrue(
                this.candle.blinkColorEndsAlliance(Canndle.GREEN, Seconds.of(0.1), Seconds.of(2.0)));
    }

    public void periodic() {
        if (DriverStation.isEnabled()) {
            limeA.setIMUMode(2);
            limeB.setIMUMode(2);
        } else {
            limeA.setIMUMode(1);
            limeB.setIMUMode(1);
        }

        limeA.setRobotOrientation(this.drivetrain.getState().Pose.getRotation());
        limeB.setRobotOrientation(this.drivetrain.getState().Pose.getRotation());
        // limeC.setRobotOrientation(this.drivetrain.getState().Pose.getRotation());

        // Right Side
        LimelightHelpers.setCameraPose_RobotSpace("limelight-b",
                Units.inchesToMeters(7.794), // Forward+
                Units.inchesToMeters(10.347), // Right+??
                Units.inchesToMeters(9.637), // Up+
                0, 0, 15.0);

        // Left Side
        // LimelightHelpers.setCameraPose_RobotSpace("limelight-c",
        // Units.inchesToMeters(7.794),
        // Units.inchesToMeters(10.347),
        // Units.inchesToMeters(9.637), 0, 0, -15.0);

        // Top Camera
        LimelightHelpers.setCameraPose_RobotSpace("limelight-a",
                Units.inchesToMeters(1.739), // Forward+
                Units.inchesToMeters(0), // Right+???
                Units.inchesToMeters(34.677), // Up+
                0, 10.118, -180.0);

        // only use reef tags for pose on low camera
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight-b",
                new int[] { 17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11 });

        // only use coral station tags for pose on back camera
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight-a", new int[] { 1, 2, 12, 13 });
    }

    public Command getAutonomousCommand() {
        // return this.drivetrain.teleopDrive(() -> 0, () -> 0.25, () ->
        // 0).repeatedly().withTimeout(3.0);
        AutonomousRoutines routines = new AutonomousRoutines(this.drivetrain);
        return routines.oneCoralFeed();
    }
}
