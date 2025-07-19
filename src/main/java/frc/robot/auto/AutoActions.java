package frc.robot.auto;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.intake.IntakePivotIO;
import frc.robot.subsystems.intake.IntakeRollerIO;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class AutoActions {
    private final DriveSubsystem driveSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public AutoActions(IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.driveSubsystem = driveSubsystem;
    }

    public Command followPath(String pathName) {
        return driveSubsystem.followPathCommand(pathName);
    }

    public Command shootCoral() {
        return new ShootCommand(intakeSubsystem);
    }
}
