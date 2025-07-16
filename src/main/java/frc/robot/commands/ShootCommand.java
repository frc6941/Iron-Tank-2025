package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.consts.Limits.Intake;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.Intaker.CurrentState;
import frc.robot.subsystems.Intaker.WantedState;

public class ShootCommand extends Command{
    private final Intaker intakeSubsystem;

    public ShootCommand(Intaker intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void execute(){
        intakeSubsystem.setWantedState(WantedState.EJECT);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setWantedState(WantedState.ZERO);
    }
}
