// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team9140.frc2025;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.team9140.frc2025.commands.AutonomousRoutines;
import org.team9140.frc2025.generated.TunerConstants;
import org.team9140.frc2025.helpers.LimelightHelpers;
import org.team9140.frc2025.subsystems.Canndle;
import org.team9140.frc2025.subsystems.CommandSwerveDrivetrain;
import org.team9140.frc2025.subsystems.Elevator;
import org.team9140.frc2025.subsystems.Funnel;
import org.team9140.frc2025.subsystems.LimeLight;
import org.team9140.frc2025.subsystems.Manipulator;

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

        LimelightHelpers.setCameraPose_RobotSpace("limelight-b",
                Units.inchesToMeters(7.5),
                Units.inchesToMeters(11.5),
                Units.inchesToMeters(13.5), 0, -12.5, 16.0);

        LimelightHelpers.setCameraPose_RobotSpace("limelight-a",
                Units.inchesToMeters(7.5),
                Units.inchesToMeters(-11.5),
                Units.inchesToMeters(13.5), 0, -9.5, -13.0);

        limeA.setIMUMode(1);
        limeB.setIMUMode(1);

        configureBindings();
    }

    private boolean stickInput() {
        return Math.abs(this.controller.getLeftX()) > 0.75
                || Math.abs(this.controller.getLeftY()) > 0.75
                || Math.abs(this.controller.getRightX()) > 0.75;
    }

    private Trigger exitAutoAlign = new Trigger(this::stickInput);

    private void configureBindings() {
        this.candle.setDefaultCommand(this.candle.solidAllianceColor());

        drivetrain
                .setDefaultCommand(
                        drivetrain.teleopDrive(controller::getLeftX, controller::getLeftY, controller::getRightX));

        controller.rightTrigger().whileTrue(this.manipulator.outtakeCoral());

        controller.rightBumper().whileTrue(
                this.manipulator.intakeCoral().alongWith(this.funnel.intakeCoral()).withName("intake coral"));
        controller.leftBumper()
                .whileTrue(this.manipulator.reverse().alongWith(this.funnel.reverse()).withName("unstick coral"));

        this.controller.y().and(this.controller.povLeft())
                .onTrue(this.drivetrain.coralReefDrive(4, true)
                        .alongWith(this.elevator.moveToPosition(Constants.Elevator.L4_coral_height))
                        .alongWith(this.candle.changeColors(Canndle.PURPLE, Canndle.OFF, 0.1))
                        .withName("coral auto score 4L")
                        .until(this::stickInput));
        this.controller.y().and(this.controller.povRight())
                .onTrue(this.drivetrain.coralReefDrive(4, false)
                        .alongWith(this.elevator.moveToPosition(Constants.Elevator.L4_coral_height))
                        .alongWith(this.candle.changeColors(Canndle.PURPLE, Canndle.OFF, 0.1))
                        .withName("coral auto score 4R")
                        .until(this::stickInput));
        this.controller.y().and(this.controller.povCenter())
                .onTrue(this.elevator.moveToPosition(Constants.Elevator.L4_coral_height));

        this.controller.b().and(this.controller.povLeft())
                .onTrue(this.drivetrain.coralReefDrive(3, true)
                        .alongWith(this.elevator.moveToPosition(Constants.Elevator.L3_coral_height))
                        .alongWith(this.candle.changeColors(Canndle.PURPLE, Canndle.OFF, 0.1))
                        .withName("coral auto score 3L")
                        .until(this::stickInput));
        this.controller.b().and(this.controller.povRight())
                .onTrue(this.drivetrain.coralReefDrive(3, false)
                        .alongWith(this.elevator.moveToPosition(Constants.Elevator.L3_coral_height))
                        .alongWith(this.candle.changeColors(Canndle.PURPLE, Canndle.OFF, 0.1))
                        .withName("coral auto score 3R")
                        .until(this::stickInput));
        this.controller.b().and(this.controller.povCenter())
                .onTrue(this.elevator.moveToPosition(Constants.Elevator.L3_coral_height));

        this.controller.a().and(this.controller.povLeft())
                .onTrue(this.drivetrain.coralReefDrive(2, true)
                        .alongWith(this.elevator.moveToPosition(Constants.Elevator.L2_coral_height))
                        .alongWith(this.candle.changeColors(Canndle.PURPLE, Canndle.OFF, 0.1))
                        .withName("coral auto score 2L")
                        .until(this::stickInput));
        this.controller.a().and(this.controller.povRight())
                .onTrue(this.drivetrain.coralReefDrive(2, false)
                        .alongWith(this.elevator.moveToPosition(Constants.Elevator.L2_coral_height))
                        .alongWith(this.candle.changeColors(Canndle.PURPLE, Canndle.OFF, 0.1))
                        .withName("coral auto score 2R")
                        .until(this::stickInput));
        this.controller.a().and(this.controller.povCenter())
                .onTrue(this.elevator.moveToPosition(Constants.Elevator.L2_coral_height));

        this.controller.x().onTrue(this.elevator.moveToPosition(Constants.Elevator.STOW_height));

        controller.start().onTrue(this.drivetrain.resetGyroCommand());

        this.elevator.isUp.onTrue(this.drivetrain.engageSlowMode()).onFalse(this.drivetrain.disengageSlowMode());

        this.exitAutoAlign.onTrue(this.candle.solidAllianceColor());

        this.drivetrain.reachedPose.onTrue(this.candle.blinkColorEndsOff(Canndle.GREEN, 0.1, 0.5));

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

        connectedTrigger.onTrue(this.candle.blinkColorEndsAlliance(Canndle.GREEN, 0.1, 2.0));
    }

    public void periodic() {
        if (DriverStation.isEnabled()) {
            // limeA.setIMUMode(2);
            limeB.setIMUMode(2);
        } else {
            // limeA.setIMUMode(1);
            limeB.setIMUMode(1);
        }
        // limeA.setRobotOrientation(this.drivetrain.getState().Pose.getRotation());
        limeB.setRobotOrientation(this.drivetrain.getState().Pose.getRotation());
        // limeC.setRobotOrientation(this.drivetrain.getState().Pose.getRotation());
    }

    public Command getAutonomousCommand() {
//        return this.drivetrain.teleopDrive(() -> 0, () -> 0.25, () -> 0).repeatedly().withTimeout(3.0);
        AutonomousRoutines routines = new AutonomousRoutines(this.drivetrain);
        return routines.threeCoral();
    }
}
