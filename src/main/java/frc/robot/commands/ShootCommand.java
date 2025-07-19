package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.WantedState;

public class ShootCommand extends Command{
    private final IntakeSubsystem intakeSubsystem;

    public ShootCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void execute(){
        intakeSubsystem.setWantedState(WantedState.HOLD_SHOOT);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setWantedState(WantedState.HOLD);
    }
}
