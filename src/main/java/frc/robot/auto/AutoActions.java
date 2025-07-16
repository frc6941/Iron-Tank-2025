package frc.robot.auto;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoActions {
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    public Command followPath(String pathName) {
        return driveSubsystem.followPathCommand(pathName);
    }

    public Command shootCoral() {
        return new ShootCommand(intakeSubsystem);
    }
}
