package org.team9140.frc2025.commands;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.team9140.frc2025.Constants;
import org.team9140.frc2025.Robot;
import org.team9140.frc2025.subsystems.CommandSwerveDrivetrain;
import org.team9140.frc2025.subsystems.Elevator;
import org.team9140.frc2025.subsystems.Funnel;
import org.team9140.frc2025.subsystems.Manipulator;
import org.team9140.lib.FollowPath;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

// TODO: put expected pose on dashboard in telemetry file
public class AutonomousRoutines {
    private static final Time INTAKE_TIME = Seconds.of(2.0);
    private static final Time THROW_TIME = Seconds.of(0.5);
    private final CommandSwerveDrivetrain drivetrain;
    private final DriverStation.Alliance alliance;

    // TODO: Align while raising elevator
    private final Supplier<Command> SCORE_CORAL_L4;
    private final Supplier<Command> RESET_ARM;
    private final Supplier<Command> INTAKE_CORAL;
    private final Supplier<Command> STOP_INTAKE;
    private final Supplier<Command> ARM_HALFWAY;
    // private final Function<Boolean, Command> REEF_DRIVE_THEN_SCORE_L4;

    public AutonomousRoutines(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

        Elevator elevator = Elevator.getInstance();
        Manipulator manipulator = Manipulator.getInstance();
        Funnel funnel = Funnel.getInstance();

        this.SCORE_CORAL_L4 = () -> elevator.moveToPosition(Constants.Elevator.L4_coral_height)
                .andThen(new WaitCommand(0.25))
                .andThen(manipulator.outtakeCoral().withTimeout(THROW_TIME));
        this.ARM_HALFWAY = () -> elevator.moveToPosition(Constants.Elevator.L1_coral_height);
        this.RESET_ARM = () -> manipulator.off().withTimeout(0.000001)
                .andThen(elevator.moveToPosition(Constants.Elevator.STOW_height));
        this.INTAKE_CORAL = () -> new WaitUntilCommand(elevator.isStowed).andThen(manipulator.intakeCoral()
                .alongWith(funnel.intakeCoral())
                .withTimeout(INTAKE_TIME));
        this.STOP_INTAKE = () -> manipulator.off().withTimeout(0.000001).alongWith(funnel.turnOff());
        // this.REEF_DRIVE_THEN_SCORE_L4 = (lefty) ->
        // drivetrain.coralReefDrive(ElevatorSetbacks.L4,
        // lefty).until(drivetrain.reachedPose).withName("final alignment")
        // .andThen(SCORE_CORAL_L4.get().deadlineFor(drivetrain.coralReefDrive(ElevatorSetbacks.L4,
        // lefty)));
    }

    public Command oneCoral() {
        FollowPath path = new FollowPath("allianceColorToJL4", () -> this.drivetrain.getState().Pose, this.drivetrain::followSample, this.alliance, drivetrain);

        return Commands.runOnce(() -> {
            if (Robot.isSimulation()) {
                this.drivetrain.resetPose(path.getInitialPose());
            }
        }).andThen(this.ARM_HALFWAY.get().alongWith(path.gimmeCommand().andThen(this.drivetrain.stop())))
                .andThen(this.SCORE_CORAL_L4.get())
                .andThen(this.RESET_ARM.get())
                .andThen(new PrintCommand("done scoring 1 coral"));
    }

    public Command oneCoralFeed() {
        FollowPath jToFeed = new FollowPath("JL4ToLeftFeed", () -> this.drivetrain.getState().Pose, this.drivetrain::followSample, this.alliance, drivetrain);
        return oneCoral()
                .andThen(jToFeed.gimmeCommand())
                .andThen(this.drivetrain.stop())
                .andThen(INTAKE_CORAL.get());
    }

    public Command hToFeed() {
        FollowPath path = new FollowPath("HL4ToRightFeed", () -> this.drivetrain.getState().Pose, this.drivetrain::followSample, this.alliance, drivetrain);
        return path.gimmeCommand().andThen(INTAKE_CORAL.get());
    }

    public Command threeCoral() {
        FollowPath farLeftToFeed = new FollowPath("JL4ToLeftFeed", () -> this.drivetrain.getState().Pose, this.drivetrain::followSample, this.alliance, drivetrain);
        FollowPath closeLeftToFeed = new FollowPath("KL4ToLeftFeed", () -> this.drivetrain.getState().Pose, this.drivetrain::followSample, this.alliance, drivetrain);
        FollowPath feedToCloseLeftLeft = new FollowPath("LeftFeedToKL4", () -> this.drivetrain.getState().Pose, this.drivetrain::followSample, this.alliance, drivetrain);
        FollowPath feedToCloseLeftRight = new FollowPath("LeftFeedToLL4", () -> this.drivetrain.getState().Pose, this.drivetrain::followSample, this.alliance, drivetrain);

        return oneCoral()
                .andThen(farLeftToFeed.gimmeCommand())
                .andThen(this.drivetrain.stop())
                .andThen(INTAKE_CORAL.get())
                .andThen(STOP_INTAKE.get())
                .andThen(feedToCloseLeftLeft.gimmeCommand())
                .andThen(this.drivetrain.stop())
                .andThen(SCORE_CORAL_L4.get())
                .andThen(RESET_ARM.get())
                .andThen(closeLeftToFeed.gimmeCommand())
                .andThen(this.drivetrain.stop())
                .andThen(INTAKE_CORAL.get())
                .andThen(STOP_INTAKE.get())
                .andThen(feedToCloseLeftRight.gimmeCommand())
                .andThen(this.drivetrain.stop())
                .andThen(SCORE_CORAL_L4.get())
                .andThen(RESET_ARM.get());
    }

    public Command testScore() {
        FollowPath testForward = new FollowPath("test_1", () -> this.drivetrain.getState().Pose, this.drivetrain::followSample, this.alliance, drivetrain);
        FollowPath testToFeed = new FollowPath("test_2", () -> this.drivetrain.getState().Pose, this.drivetrain::followSample, this.alliance, drivetrain);
        FollowPath feedToScore = new FollowPath("test_3", () -> this.drivetrain.getState().Pose, this.drivetrain::followSample, this.alliance, drivetrain);

        return Commands.runOnce(() -> this.drivetrain.resetPose(testForward.getInitialPose()))
                .andThen(testForward.gimmeCommand());
    }
}