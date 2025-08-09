package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;

import static edu.wpi.first.units.Units.Rotations;

public class PivotUpCommand extends Command {
    public IntakeSubsystem intakeSubsystem;

    public PivotUpCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setPivotPosition(Rotations.of(RobotConstants.IntakeConstants.ELEVATED_POSITION_DEGREES.get()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
