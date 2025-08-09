package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

public class IntakerCommand extends Command {
    public IntakeSubsystem intakeSubsystem;

    public IntakerCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intakeSubsystem.setRollerVoltage(Volts.of(RobotConstants.IntakeConstants.INTAKE_VOLTAGE.get()));
        intakeSubsystem.setPivotPosition(Rotations.of(RobotConstants.IntakeConstants.INTAKE_POSITION_DEGREES.get()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setPivotPosition(Rotations.of(RobotConstants.IntakeConstants.ELEVATED_POSITION_DEGREES.get()));
    }
}
