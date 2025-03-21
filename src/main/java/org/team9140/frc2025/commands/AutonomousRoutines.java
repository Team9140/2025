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
                .andThen(new WaitCommand(0.2))
                .andThen(manipulator.outtakeCoral().withTimeout(THROW_TIME));
        this.ARM_HALFWAY = () -> elevator.moveToPosition(Constants.Elevator.L1_coral_height);
        this.RESET_ARM = () -> manipulator.turnOff()
                .andThen(elevator.moveToPosition(Constants.Elevator.STOW_height));
        this.INTAKE_CORAL = () -> new WaitUntilCommand(elevator.isStowed).andThen(manipulator.intakeCoral()
                .alongWith(funnel.intakeCoral())
                .withTimeout(INTAKE_TIME));
        this.STOP_INTAKE = () -> manipulator.turnOff().alongWith(funnel.turnOff());
        // this.REEF_DRIVE_THEN_SCORE_L4 = (lefty) ->
        // drivetrain.coralReefDrive(ElevatorSetbacks.L4,
        // lefty).until(drivetrain.reachedPose).withName("final alignment")
        // .andThen(SCORE_CORAL_L4.get().deadlineFor(drivetrain.coralReefDrive(ElevatorSetbacks.L4,
        // lefty)));
    }

    public Command oneCoral() {
        FollowPath path = new FollowPath("oneCoral", drivetrain, alliance);

        return Commands.runOnce(() -> {
            if (Robot.isSimulation()) {
                this.drivetrain.resetPose(path.getInitialPose());
            }
        }).andThen(this.ARM_HALFWAY.get().alongWith(path.gimmeCommand()))
                .andThen(SCORE_CORAL_L4.get())
                .andThen(RESET_ARM.get())
                .andThen(new PrintCommand("done scoring 1 coral"));
    }

    public Command oneCoralFeed() {
        return oneCoral().andThen(hToFeed());
    }

    public Command hToFeed() {
        FollowPath path = new FollowPath("hToFeed", drivetrain, alliance);
        return path.gimmeCommand().andThen(INTAKE_CORAL.get());
    }

    public Command threeCoral() {
        FollowPath farLeftToFeed = new FollowPath("farLeftToFeed", drivetrain, alliance);
        FollowPath closeLeftToFeed = new FollowPath("closeLeftToFeed", drivetrain, alliance);
        FollowPath feedToCloseLeftLeft = new FollowPath("feedToCloseLeftLeft", drivetrain, alliance);
        FollowPath feedToCloseLeftRight = new FollowPath("feedToCloseLeftRight", drivetrain, alliance);

        return oneCoral()
                .andThen(farLeftToFeed.gimmeCommand())
                .andThen(INTAKE_CORAL.get())
                .andThen(STOP_INTAKE.get())
                .andThen(feedToCloseLeftLeft.gimmeCommand())
                .andThen(SCORE_CORAL_L4.get())
                .andThen(RESET_ARM.get())
                .andThen(closeLeftToFeed.gimmeCommand())
                .andThen(INTAKE_CORAL.get())
                .andThen(STOP_INTAKE.get())
                .andThen(feedToCloseLeftRight.gimmeCommand())
                .andThen(SCORE_CORAL_L4.get())
                .andThen(RESET_ARM.get());
    }

    public Command testScore() {
        FollowPath testForward = new FollowPath("test_1", drivetrain, alliance);
        FollowPath testToFeed = new FollowPath("test_2", drivetrain, alliance);
        FollowPath feedToScore = new FollowPath("test_3", drivetrain, alliance);

        return Commands.runOnce(() -> this.drivetrain.resetPose(testForward.getInitialPose()))
                .andThen(this.ARM_HALFWAY.get().alongWith(testForward.gimmeCommand()))
                .andThen(SCORE_CORAL_L4.get())
                .andThen(RESET_ARM.get())
                .andThen(testToFeed.gimmeCommand())
                .andThen(INTAKE_CORAL.get())
                .andThen(STOP_INTAKE.get())
                .andThen(feedToScore.gimmeCommand())
                .andThen(SCORE_CORAL_L4.get())
                .andThen(RESET_ARM.get());
    }
}