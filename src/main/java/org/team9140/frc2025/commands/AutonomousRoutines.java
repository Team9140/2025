package org.team9140.frc2025.commands;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team9140.frc2025.Constants;
import org.team9140.frc2025.Constants.ElevatorSetbacks;
import org.team9140.frc2025.helpers.AutoAiming;
import org.team9140.frc2025.subsystems.CommandSwerveDrivetrain;
import org.team9140.frc2025.subsystems.Elevator;
import org.team9140.frc2025.subsystems.Funnel;
import org.team9140.frc2025.subsystems.Manipulator;
import org.team9140.lib.FollowPath;
import org.team9140.lib.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// TODO: put expected pose on dashboard in telemetry file
public class AutonomousRoutines {
    private static final Time INTAKE_TIME = Seconds.of(1.0);
    private static final Time THROW_TIME = Seconds.of(0.5);
    private final CommandSwerveDrivetrain drivetrain;

    // TODO: Align while raising elevator
    // private final Supplier<Command> SCORE_CORAL_L4;
    private final Supplier<Command> INTAKE_CORAL;
    private final Supplier<Command> STOP_INTAKE;
    // private final Supplier<Command> ARM_HALFWAY;
    // private final Function<Boolean, Command> REEF_DRIVE_THEN_SCORE_L4;

    Elevator elevator = Elevator.getInstance();
    Manipulator manipulator = Manipulator.getInstance();
    Funnel funnel = Funnel.getInstance();

    public AutonomousRoutines(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // this.SCORE_CORAL_L4 = () ->
        // elevator.moveToPosition(Constants.Elevator.L4_coral_height)
        // .andThen(new WaitCommand(0.25))
        // .andThen(manipulator.outtakeCoral().withTimeout(THROW_TIME));
        // this.ARM_HALFWAY = () ->
        // elevator.moveToPosition(Constants.Elevator.L1_coral_height);
        this.INTAKE_CORAL = () -> manipulator.intakeCoral().alongWith(funnel.intakeCoral());
        this.STOP_INTAKE = () -> manipulator.off().withTimeout(0.000001).alongWith(funnel.turnOff());
        // this.REEF_DRIVE_THEN_SCORE_L4 = (lefty) ->
        // drivetrain.coralReefDrive(ElevatorSetbacks.L4,
        // lefty).until(drivetrain.reachedPose).withName("final alignment")
        // .andThen(SCORE_CORAL_L4.get().deadlineFor(drivetrain.coralReefDrive(ElevatorSetbacks.L4,
        // lefty)));
    }

    public Command oneCoralCenter() {
        // score on G
        Pose2d scorePose;
        if (Util.getAlliance().equals(Optional.of(Alliance.Blue))) {
            scorePose = AutoAiming.ReefFaces.GH_B.getLeft(ElevatorSetbacks.L4);
        } else {
            scorePose = AutoAiming.ReefFaces.GH_R.getLeft(ElevatorSetbacks.L4);
        }

        return new WaitCommand(Seconds.of(5.0)).andThen(this.drivetrain.goToPose(() -> scorePose)
                .until(this.drivetrain.reachedPose)
                .alongWith(this.elevator.moveToPosition(Constants.Elevator.L4_coral_height))
                .andThen(this.drivetrain.stop())
                .andThen(manipulator.outtakeCoral().withTimeout(THROW_TIME))
                .andThen(this.elevator.moveToPosition(Constants.Elevator.STOW_height)));
    }

    public Command oneCoralInsideLeft() {
        // score on J
        Pose2d scorePose;
        if (Util.getAlliance().equals(Optional.of(Alliance.Blue))) {
            scorePose = AutoAiming.ReefFaces.IJ_B.getLeft(ElevatorSetbacks.L4);
        } else {
            scorePose = AutoAiming.ReefFaces.IJ_R.getLeft(ElevatorSetbacks.L4);
        }

        return this.drivetrain.goToPose(() -> scorePose)
                .until(this.drivetrain.reachedPose)
                .alongWith(this.elevator.moveToPosition(Constants.Elevator.L4_coral_height))
                .andThen(this.drivetrain.stop())
                .andThen(manipulator.outtakeCoral().withTimeout(THROW_TIME))
                .andThen(this.elevator.moveToPosition(Constants.Elevator.STOW_height));
    }

    public Command oneCoralInsideRight() {
        // score on E
        Pose2d scorePose;
        if (Util.getAlliance().equals(Optional.of(Alliance.Blue))) {
            scorePose = AutoAiming.ReefFaces.EF_B.getRight(ElevatorSetbacks.L4);
        } else {
            scorePose = AutoAiming.ReefFaces.EF_R.getRight(ElevatorSetbacks.L4);
        }

        return this.drivetrain.goToPose(() -> scorePose)
                .until(this.drivetrain.reachedPose)
                .alongWith(this.elevator.moveToPosition(Constants.Elevator.L4_coral_height))
                .andThen(this.drivetrain.stop())
                .andThen(manipulator.outtakeCoral().withTimeout(THROW_TIME))
                .andThen(this.elevator.moveToPosition(Constants.Elevator.STOW_height));
    }

    private Command JtoLeftFeed() {
        FollowPath path = new FollowPath("JL4ToLeftFeed", () -> this.drivetrain.getState().Pose,
                this.drivetrain::followSample, Util.getAlliance().get(), drivetrain);
        return path.gimmeCommand();
    }

    private Command EtoRightFeed() {
        FollowPath path = new FollowPath("EL4ToRightFeed", () -> this.drivetrain.getState().Pose,
                this.drivetrain::followSample, Util.getAlliance().get(), drivetrain);
        return path.gimmeCommand();
    }

    public Command twoCoralInsideRight() {
        // score on E
        Pose2d scorePose;
        if (Util.getAlliance().equals(Optional.of(Alliance.Blue))) {
            scorePose = AutoAiming.ReefFaces.CD_B.getRight(ElevatorSetbacks.L4);
        } else {
            scorePose = AutoAiming.ReefFaces.CD_R.getRight(ElevatorSetbacks.L4);
        }

        return this.oneCoralInsideRight()
                .andThen(this.INTAKE_CORAL.get().raceWith(this.EtoRightFeed())
                        .andThen(this.INTAKE_CORAL.get().withTimeout(INTAKE_TIME))
                        .andThen(drivetrain.goToPose(() -> scorePose)
                                .alongWith(this.INTAKE_CORAL.get())
                                .until(this.drivetrain.reachedPose)
                                .andThen(this.drivetrain.stop())))
                .andThen(this.STOP_INTAKE.get())
                .andThen(this.elevator.moveToPosition(Constants.Elevator.L4_coral_height))
                .andThen(manipulator.outtakeCoral().withTimeout(THROW_TIME))
                .andThen(this.elevator.moveToPosition(Constants.Elevator.STOW_height));
    }

    public Command twoCoralInsideLeft() {
        // score on K
        Pose2d scorePose;
        if (Util.getAlliance().equals(Optional.of(Alliance.Blue))) {
            scorePose = AutoAiming.ReefFaces.KL_B.getLeft(ElevatorSetbacks.L4);
        } else {
            scorePose = AutoAiming.ReefFaces.KL_R.getLeft(ElevatorSetbacks.L4);
        }

        return this.oneCoralInsideLeft()
                .andThen(this.INTAKE_CORAL.get().raceWith(this.JtoLeftFeed())
                        .andThen(this.INTAKE_CORAL.get().withTimeout(INTAKE_TIME))
                        .andThen(drivetrain.goToPose(() -> scorePose)
                                .alongWith(this.INTAKE_CORAL.get())
                                .until(this.drivetrain.reachedPose)
                                .andThen(this.drivetrain.stop())))
                .andThen(this.STOP_INTAKE.get())
                .andThen(this.elevator.moveToPosition(Constants.Elevator.L4_coral_height))
                .andThen(this.manipulator.outtakeCoral().withTimeout(THROW_TIME))
                .andThen(this.elevator.moveToPosition(Constants.Elevator.STOW_height));
    }

    // public Command oneCoralFeed() {
    // FollowPath jToFeed = new FollowPath("JL4ToLeftFeed", () ->
    // this.drivetrain.getState().Pose,
    // this.drivetrain::followSample, this.alliance, drivetrain);
    // return oneCoral()
    // .andThen(jToFeed.gimmeCommand())
    // .andThen(this.drivetrain.stop())
    // .andThen(INTAKE_CORAL.get());
    // }

    // public Command hToFeed() {
    // FollowPath path = new FollowPath("HL4ToRightFeed", () ->
    // this.drivetrain.getState().Pose,
    // this.drivetrain::followSample, this.alliance, drivetrain);
    // return path.gimmeCommand().andThen(INTAKE_CORAL.get());
    // }

    // public Command threeCoral() {
    // FollowPath JL4ToLeftFeed = new FollowPath("JL4ToLeftFeed", () ->
    // this.drivetrain.getState().Pose,
    // this.drivetrain::followSample, this.alliance, drivetrain);
    // FollowPath LeftFeedToKL4 = new FollowPath("LeftFeedToKL4", () ->
    // this.drivetrain.getState().Pose,
    // this.drivetrain::followSample, this.alliance, drivetrain);
    // FollowPath KL4ToLeftFeed = new FollowPath("KL4ToLeftFeed", () ->
    // this.drivetrain.getState().Pose,
    // this.drivetrain::followSample, this.alliance, drivetrain);
    // FollowPath LeftFeedToLL4 = new FollowPath("LeftFeedToLL4", () ->
    // this.drivetrain.getState().Pose,
    // this.drivetrain::followSample, this.alliance, drivetrain);

    // return oneCoral()
    // .andThen(JL4ToLeftFeed.gimmeCommand())
    // .andThen(this.drivetrain.stop())
    // .andThen(INTAKE_CORAL.get())
    // .andThen(STOP_INTAKE.get())
    // .andThen(LeftFeedToKL4.gimmeCommand())
    // .andThen(this.drivetrain.stop())
    // .andThen(SCORE_CORAL_L4.get())
    // .andThen(STOW_ARM.get())
    // .andThen(KL4ToLeftFeed.gimmeCommand())
    // .andThen(this.drivetrain.stop())
    // .andThen(INTAKE_CORAL.get())
    // .andThen(STOP_INTAKE.get())
    // .andThen(LeftFeedToLL4.gimmeCommand())
    // .andThen(this.drivetrain.stop())
    // .andThen(SCORE_CORAL_L4.get())
    // .andThen(STOW_ARM.get());
    // }

    // public Command testScore() {
    // FollowPath testForward = new FollowPath("test_1", () ->
    // this.drivetrain.getState().Pose,
    // this.drivetrain::followSample, this.alliance, drivetrain);
    // FollowPath testToFeed = new FollowPath("test_2", () ->
    // this.drivetrain.getState().Pose,
    // this.drivetrain::followSample, this.alliance, drivetrain);
    // FollowPath feedToScore = new FollowPath("test_3", () ->
    // this.drivetrain.getState().Pose,
    // this.drivetrain::followSample, this.alliance, drivetrain);

    // return Commands.runOnce(() ->
    // this.drivetrain.resetPose(testForward.getInitialPose()))
    // .andThen(testForward.gimmeCommand());
    // }
}