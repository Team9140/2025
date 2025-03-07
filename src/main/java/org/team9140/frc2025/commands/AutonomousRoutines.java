package org.team9140.frc2025.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team9140.frc2025.Constants;
import org.team9140.frc2025.subsystems.CommandSwerveDrivetrain;
import org.team9140.frc2025.subsystems.Elevator;
import org.team9140.frc2025.subsystems.Manipulator;
import org.team9140.lib.FollowPath;

public class AutonomousRoutines {
    private static final Command SCORE_CORAL_L4 = Elevator.getInstance().moveToPosition(Constants.Elevator.L4_coral_height).andThen(new WaitCommand(5.0).deadlineFor(Manipulator.getInstance().outtakeCoral()));
    private static final Command RESET_ARM = Manipulator.getInstance().turnOff().andThen(Elevator.getInstance().moveToPosition(Constants.Elevator.STOW_height));
    private static final Command INTAKE_CORAL = Manipulator.getInstance().intakeCoral();

    public static Command oneCoral(CommandSwerveDrivetrain drivetrain) {
        FollowPath path = new FollowPath("oneCoral", drivetrain, DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
        return path.gimmeCommand().andThen(new WaitCommand(5.0).deadlineFor(SCORE_CORAL_L4)).andThen(RESET_ARM);
    }

    public static Command feedOneCoral(CommandSwerveDrivetrain drivetrain) {
        return oneCoral(drivetrain).andThen(feed(drivetrain));
    }

    public static Command feed(CommandSwerveDrivetrain drivetrain) {
        FollowPath path = new FollowPath("feed", drivetrain, DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
        return path.gimmeCommand().andThen(INTAKE_CORAL);
    }
}