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
    private static final Time INTAKE_TIME = Seconds.of(1);
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
                .andThen(new WaitCommand(3.0).deadlineFor(Manipulator.getInstance().outtakeCoral()));
        this.RESET_ARM = () -> Manipulator.getInstance().turnOff()
                .andThen(Elevator.getInstance().moveToPosition(Constants.Elevator.STOW_height));
        this.INTAKE_CORAL = () -> new WaitCommand(INTAKE_TIME)
                .deadlineFor(Manipulator.getInstance().intakeCoral().alongWith(Funnel.getInstance().intakeCoral()));
        this.STOP_INTAKE = () -> Manipulator.getInstance().turnOff().alongWith(Funnel.getInstance().turnOff());
    }

    public Command oneCoral() {
        FollowPath path = new FollowPath("oneCoral", drivetrain, alliance);
        return path.gimmeCommand()
                .andThen(SCORE_CORAL_L4.get())
                .andThen(RESET_ARM.get())
                .andThen(new PrintCommand("test"));
    }

    public Command oneCoralFeed() {
        return oneCoral().andThen(hToFeed());
    }

    public Command hToFeed() {
        FollowPath path = new FollowPath("hToFeed", drivetrain, alliance);
        return path.gimmeCommand().andThen(INTAKE_CORAL.get());
    }

    public Command threeCoral() {
        FollowPath feedToD = new FollowPath("feedToD", drivetrain, alliance);
        FollowPath dToFeed = new FollowPath("dToFeed", drivetrain, alliance);
        FollowPath feedToC = new FollowPath("feedToC", drivetrain, alliance);

        return oneCoralFeed()
                .andThen(STOP_INTAKE.get())
                .andThen(feedToD.gimmeCommand())
                .andThen(SCORE_CORAL_L4.get())
                .andThen(RESET_ARM.get())
                .andThen(dToFeed.gimmeCommand())
                .andThen(INTAKE_CORAL.get())
                .andThen(STOP_INTAKE.get())
                .andThen(feedToC.gimmeCommand())
                .andThen(SCORE_CORAL_L4.get())
                .andThen(RESET_ARM.get());
    }
}