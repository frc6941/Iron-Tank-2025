package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.consts.Limits.Intake;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.CurrentState;
import frc.robot.subsystems.IntakeSubsystem.WantedState;

public class ShootCommand extends Command{
    private final IntakeSubsystem intakeSubsystem;

    public ShootCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void execute(){
        intakeSubsystem.setWantedState(WantedState.EJECT);
    }

    @Override
    public boolean isFinished(){
        return intakeSubsystem.isShootFinished();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setWantedState(WantedState.ZERO);
    }
}
