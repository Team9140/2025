package org.team9140.frc2025.commands;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team9140.frc2025.Constants;
import org.team9140.frc2025.subsystems.CommandSwerveDrivetrain;
import org.team9140.frc2025.subsystems.Elevator;
import org.team9140.frc2025.subsystems.Funnel;
import org.team9140.frc2025.subsystems.Manipulator;
import org.team9140.lib.FollowPath;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Seconds;

public class AutonomousRoutines {
    private static final Time INTAKE_TIME = Seconds.of(2);
    private static final Time THROW_TIME = Seconds.of(0.5);
    private final CommandSwerveDrivetrain drivetrain;
    private final DriverStation.Alliance alliance;

    private final Supplier<Command> SCORE_CORAL_L4;
    private final Supplier<Command> RESET_ARM;
    private final Supplier<Command> INTAKE_CORAL;
    private final Supplier<Command> STOP_INTAKE;

    public AutonomousRoutines(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        this.SCORE_CORAL_L4 = () -> Elevator.getInstance().moveToPosition(Constants.Elevator.L4_coral_height)
                .andThen(new WaitCommand(2.0))
                .andThen(Manipulator.getInstance().outtakeCoral().withTimeout(THROW_TIME));
        this.RESET_ARM = () -> Manipulator.getInstance().turnOff()
                .andThen(Elevator.getInstance().moveToPosition(Constants.Elevator.STOW_height));
        this.INTAKE_CORAL = () -> Manipulator.getInstance().intakeCoral()
                .alongWith(Funnel.getInstance().intakeCoral())
                .withTimeout(INTAKE_TIME);
        this.STOP_INTAKE = () -> Manipulator.getInstance().turnOff().alongWith(Funnel.getInstance().turnOff());
    }

    public Command oneCoral() {
        FollowPath path = new FollowPath("oneCoral", drivetrain, alliance);
        return path.gimmeCommand()
                .andThen(this.drivetrain.stop())
                .andThen(new PrintCommand("finished path"))
                .andThen(drivetrain.coralReefDrive(4, false).withName("final alignment")).withTimeout(Seconds.of(2))
                .andThen(drivetrain.stop())
                .andThen(SCORE_CORAL_L4.get())
                .andThen(RESET_ARM.get())
                .andThen(new PrintCommand("done scoring 1 coral"));
    }

    public Command oneCoralFeed() {
        return oneCoral().andThen(hToFeed()).andThen(drivetrain.stop());
    }

    public Command hToFeed() {
        FollowPath path = new FollowPath("hToFeed", drivetrain, alliance);
        return path.gimmeCommand().andThen(INTAKE_CORAL.get());
    }

    public Command threeCoral() {
        FollowPath feedToCloseRight = new FollowPath("feedToCloseRight", drivetrain, alliance);
        FollowPath closeRightToFeed = new FollowPath("closeRightToFeed", drivetrain, alliance);

        return oneCoralFeed()
                .andThen(STOP_INTAKE.get())
                .andThen(feedToCloseRight.gimmeCommand())
                .andThen(drivetrain.coralReefDrive(4, true).until(drivetrain.reachedPose::getAsBoolean)
                        .withName("final alignment"))
                .andThen(this.drivetrain.stop())
                .andThen(SCORE_CORAL_L4.get())
                .andThen(RESET_ARM.get())
                .andThen(closeRightToFeed.gimmeCommand())
                .andThen(INTAKE_CORAL.get())
                .andThen(STOP_INTAKE.get())
                .andThen(feedToCloseRight.gimmeCommand())
                .andThen(drivetrain.coralReefDrive(4, false).until(drivetrain.reachedPose::getAsBoolean)
                        .withName("final alignment"))
                .andThen(this.drivetrain.stop())
                .andThen(SCORE_CORAL_L4.get())
                .andThen(RESET_ARM.get());
    }

    public Command testScore() {
        return this.SCORE_CORAL_L4.get().andThen(this.RESET_ARM.get());
    }
}