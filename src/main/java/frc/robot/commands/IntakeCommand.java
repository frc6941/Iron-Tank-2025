package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;

import static edu.wpi.first.units.Units.Volts;

public class IntakeCommand extends Command {
    public IntakeSubsystem intakeSubsystem;

    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        intakeSubsystem.setRollerVoltage(Volts.of(RobotConstants.IntakeConstants.INTAKE_VOLTAGE.get()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setRollerVoltage(Volts.of(0));
    }
}
