package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.RobotConstants.IntakeConstants.IntakeMovePivotPID.*;


public class PivotDownCommand extends Command {
    IntakeSubsystem m_IntakeSubsystem;
    PIDController pivotPIDCtrl = new PIDController(kP.get(), kI.get(), kD.get());
    double targetAngle;
    double position;
    CommandXboxController Controller;

    public PivotDownCommand(IntakeSubsystem intakeSubsystem, double position, double targetAngle) {
        this.position = position;
        this.m_IntakeSubsystem = intakeSubsystem;
        this.targetAngle = targetAngle;
        pivotPIDCtrl.reset();
        pivotPIDCtrl.setSetpoint(targetAngle);
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        if (RobotConstants.TUNING) {
            pivotPIDCtrl.setPID(kP.get(), kI.get(), kD.get());
        }
    }

    @Override
    public void execute() {
        m_IntakeSubsystem.setRollerVoltage(Volts.of(RobotConstants.IntakeConstants.INTAKE_VOLTAGE.get()));
        m_IntakeSubsystem.setPivotVoltage(
                Volts.of(
                        pivotPIDCtrl.calculate(
                                m_IntakeSubsystem.getPivotPosition()
                        )
                )
        );
        Logger.recordOutput("Tank/targetAngle", targetAngle);
        Logger.recordOutput("Tank/measuredAngle", m_IntakeSubsystem.getPivotPosition());
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.setPivotPosition(Rotations.of(RobotConstants.IntakeConstants.ELEVATED_POSITION_DEGREES.get()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}