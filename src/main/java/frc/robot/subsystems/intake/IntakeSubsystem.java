package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SuperstructureVisualizer;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerSubsystem;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.Superstructures.Intake.*;

import java.util.function.Supplier;

public class IntakeSubsystem extends RollerSubsystem {
    public static final String NAME = "Intake/Roller";
    private static double holdAngle = HOLD_ANGLE.get();
    private static double deployAngle = DEPLOY_ANGLE.get();
    private static double homeAngle = HOME_ANGLE.get();
    private static double shootVoltage = SHOOT_VOLTAGE.get();
    private static double holdVoltage = HOLD_VOLTAGE.get();
    private static double intakeVoltage = INTAKE_VOLTAGE.get();
    private static double rollerHasCoralAmps = HAS_CORAL_CURRENT_THRESHOLD.get();
    private static double rollerHasCoralVel = HAS_CORAL_VELOCITY_THRESHOLD.get();
    private final IntakePivotIO intakePivotIO;
    private final IntakeRollerIO intakeRollerIO;
    private final IntakePivotIOInputsAutoLogged intakePivotIOInputs = new IntakePivotIOInputsAutoLogged();
    private final RollerIOInputsAutoLogged intakeRollerIOInputs = new RollerIOInputsAutoLogged();
    private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
    private WantedState wantedState = WantedState.HOME;
    private SystemState systemState = SystemState.HOMING;
    private double currentFilterValue = 0.0;

    public IntakeSubsystem(
            IntakePivotIO intakePivotIO,
            IntakeRollerIO intakeRollerIO
    ) {
        super(intakeRollerIO, NAME);
        this.intakePivotIO = intakePivotIO;
        this.intakeRollerIO = intakeRollerIO;
    }

    @Override
    public void periodic() {
        super.periodic();

        intakePivotIO.updateInputs(intakePivotIOInputs);

        SystemState newState = handleStateTransition();

        Logger.processInputs("Intake/Pivot", intakePivotIOInputs);
        Logger.recordOutput("Intake/SystemState", systemState.toString());
        Logger.recordOutput("Intake/WantedState", wantedState.toString());

        SuperstructureVisualizer.getInstance().updateIntake(intakePivotIOInputs.currentAngleDeg);

        currentFilterValue = currentFilter.calculate(intakePivotIOInputs.statorCurrentAmps);
        Logger.recordOutput("Intake/StatorCurrent", currentFilterValue);

        if (newState != systemState) {
            systemState = newState;
        }

        switch (systemState) {
            case DEPLOY_WITHOUT_ROLLING:
                intakeRollerIO.stop();
                intakePivotIO.setPivotAngle(deployAngle);
                break;
            case DEPLOY_INTAKING:
                intakeRollerIO.setVoltage(intakeVoltage);
                intakePivotIO.setPivotAngle(deployAngle);
                break;
            case OUTTAKING:
                intakeRollerIO.setVoltage(-intakeVoltage);
                intakePivotIO.setPivotAngle(deployAngle);
                break;
            case HOMING:
                intakeRollerIO.stop();
                intakePivotIO.setPivotAngle(homeAngle);
                break;
            case HOLDING:
                intakeRollerIO.setVoltage(holdVoltage);
                intakePivotIO.setPivotAngle(holdAngle);
                break;
            case HOLD_SHOOTING:
                intakeRollerIO.setVoltage(shootVoltage);
                intakePivotIO.setPivotAngle(holdAngle);
                break;
            case OFF:
                break;
        }

        if (Constants.TUNING) {
            deployAngle = DEPLOY_ANGLE.get();
            shootVoltage = SHOOT_VOLTAGE.get();
            homeAngle = HOME_ANGLE.get();
            intakeVoltage = INTAKE_VOLTAGE.get();
            rollerHasCoralAmps = HAS_CORAL_CURRENT_THRESHOLD.get();
            rollerHasCoralVel = HAS_CORAL_VELOCITY_THRESHOLD.get();
            holdAngle = HOLD_ANGLE.get();
            holdVoltage = HOLD_VOLTAGE.get();
        }
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case DEPLOY_WITHOUT_ROLL -> SystemState.DEPLOY_WITHOUT_ROLLING;
            case DEPLOY_INTAKE -> SystemState.DEPLOY_INTAKING;
            case OUTTAKE -> SystemState.OUTTAKING;
            case HOME -> SystemState.HOMING;
            case HOLD -> SystemState.HOLDING;
            case HOLD_SHOOT -> SystemState.HOLD_SHOOTING;
            case OFF -> SystemState.OFF;
        };
    }

    public boolean hasCoral() {
        // If the roller does not have a voltage applied, it is impossible to hold a coral so we short circuit this check
        if (intakeRollerIOInputs.appliedVolts < intakeVoltage / 2) {
            return false;
        }

        if (intakeRollerIOInputs.velocityRotPerSec < rollerHasCoralVel && intakeRollerIOInputs.statorCurrentAmps > rollerHasCoralAmps) {
            return true;
        }

        return false;
    }

    public boolean isNearAngle(double targetAngleDeg) {
        return MathUtil.isNear(targetAngleDeg, intakePivotIOInputs.currentAngleDeg, 1);
        //TODO tolerance ++
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public Command setWantedState(Supplier<WantedState> wantedState) {
        return run(() -> setWantedState(wantedState.get()));
    }

    public SystemState getSystemState() {
        return systemState;
    }

    public enum WantedState {
        DEPLOY_WITHOUT_ROLL,
        DEPLOY_INTAKE,
        OUTTAKE,
        HOLD,
        HOLD_SHOOT,
        HOME,
        OFF,
    }

    public enum SystemState {
        DEPLOY_WITHOUT_ROLLING,
        DEPLOY_INTAKING,
        OUTTAKING,
        HOLDING,
        HOLD_SHOOTING,
        HOMING,
        OFF,
    }
}