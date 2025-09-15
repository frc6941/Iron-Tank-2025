package frc.robot.subsystems.Intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Roller.RollerIO;
import frc.robot.subsystems.Roller.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;
import frc.robot.commands.PivotDownCommand;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class IntakeSubsystem extends SubsystemBase {
    private RollerIO rollerIO;
    private PivotIO pivotIO;
    private RollerIOInputsAutoLogged rollerIOInputsAutoLogged = new RollerIOInputsAutoLogged();
    private PivotIOInputsAutoLogged pivotIOInputsAutoLogged = new PivotIOInputsAutoLogged();

    public IntakeSubsystem(RollerIO rollerIO, PivotIO piviotIO) {
        this.pivotIO = piviotIO;
        this.rollerIO = rollerIO;
    }

    @Override
    public void periodic() {
        rollerIO.updateInputs(rollerIOInputsAutoLogged);
        pivotIO.updateInputs(pivotIOInputsAutoLogged);
        Logger.processInputs("Intake/Roller", rollerIOInputsAutoLogged);
        Logger.processInputs("Intake/Pivot", pivotIOInputsAutoLogged);

        if (RobotConstants.TUNING) {
            pivotIO.updateConfigs(
                    RobotConstants.IntakeConstants.IntakeMovePivotPID.kP.get(),
                    RobotConstants.IntakeConstants.IntakeMovePivotPID.kI.get(),
                    RobotConstants.IntakeConstants.IntakeMovePivotPID.kD.get(),
                    RobotConstants.IntakeConstants.IntakeMovePivotPID.kA.get(),
                    RobotConstants.IntakeConstants.IntakeMovePivotPID.kV.get(),
                    RobotConstants.IntakeConstants.IntakeMovePivotPID.kS.get(),
                    RobotConstants.IntakeConstants.IntakeMovePivotPID.kG.get()
            );
            rollerIO.updateConfigs(
                    RobotConstants.IntakeConstants.IntakeRollerPID.kP.get(),
                    RobotConstants.IntakeConstants.IntakeRollerPID.kI.get(),
                    RobotConstants.IntakeConstants.IntakeRollerPID.kD.get(),
                    RobotConstants.IntakeConstants.IntakeRollerPID.kA.get(),
                    RobotConstants.IntakeConstants.IntakeRollerPID.kV.get(),
                    RobotConstants.IntakeConstants.IntakeRollerPID.kS.get()
            );
        }
    }

    public double getPivotPosition() {
        return pivotIOInputsAutoLogged.currentPositionRot;
    }

    public void setPivotPosition(Angle position) {
        pivotIO.setPosition(position);
    }

    public void setPivotVoltage(Voltage voltage) {
        pivotIO.setVoltage(voltage.in(Volts));
    }

    public void setRollerVoltage(Voltage voltage) {
        rollerIO.setVoltage(voltage.in(Volts));
    }

    public void setRollerVelocity(AngularVelocity velocity) {
        rollerIO.setVelocity(velocity.in(RotationsPerSecond));
    }
}