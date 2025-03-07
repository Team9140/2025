package org.team9140.frc2025.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team9140.frc2025.Constants;
import org.team9140.frc2025.subsystems.CommandSwerveDrivetrain;
import org.team9140.frc2025.subsystems.Elevator;
import org.team9140.frc2025.subsystems.Manipulator;
import org.team9140.lib.MazeRunner;

public class AutonomousRoutines {
    private static final Command SCORE_CORAL_L1 = Elevator.getInstance().moveToPosition(Constants.Elevator.L1).andThen(Manipulator.getInstance().outtakeCoral());
    private static final Command RESET_ARM = Manipulator.getInstance().turnOff().andThen(Elevator.getInstance().moveToPosition(Constants.Elevator.BOTTOM));

    public static Command oneCoral(CommandSwerveDrivetrain drivetrain) {
        MazeRunner path = new MazeRunner("oneCoral", drivetrain, DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
        return path.gimmeCommand().andThen(new WaitCommand(5.0).deadlineFor(SCORE_CORAL_L1)).andThen(RESET_ARM);
    }
}
