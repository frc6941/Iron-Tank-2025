package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.consts;
import frc.robot.SuperstructureVisualizer;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerSubsystem;
import org.littletonrobotics.junction.Logger;

import static frc.robot.consts.Superstructures.Intake.*;

public class IntakeSubsystem extends RollerSubsystem {
    public static final String NAME = "Intake/Roller";
    private static double shootAngle = EJECT_POSITION.get();
    private static double deployAngle = ELEVATED_POSITION.get();
    private static double shootVoltage = ROLLER_VOLTAGE.get();
    private static double homeAngle = HOME_POSITION.get();
    private static double intakeVoltage = INTAKE_VOLTAGE.get();
    private static double intakeHoldVoltage = INTAKE_HOLD_VOLTAGE.get();
    private static double rollerAmpsHasCoral = HAS_CORAL_CURRENT_THRESHOLD.get();
    private static double intakeTime = 1.0;
    private final IntakePivotIO intakePivotIO;
    private final IntakeRollerIO intakeRollerIO;
    private final IntakePivotIOInputsAutoLogged intakePivotIOInputs = new IntakePivotIOInputsAutoLogged();
    private final RollerIOInputsAutoLogged intakeRollerIOInputs = new RollerIOInputsAutoLogged();
    private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
    public boolean hasHomed = false;
    Timer timer = new Timer();
    private boolean shouldOuttake = false;
    private WantedState wantedState = WantedState.HOME;
    private SystemState systemState = SystemState.HOMING;
    private double currentFilterValue = 0.0;
    private boolean timerStarted = false;
    private boolean lowerAngle = false;

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
            case SHOOTING:
                intakeRollerIO.setVoltage(shootVoltage);
                intakePivotIO.setPivotAngle(shootAngle);
                break;
            case DEPLOY_SHOOTING:
                intakeRollerIO.setVoltage(0);
                intakePivotIO.setPivotAngle(shootAngle);
                break;
            case HOMING:
                intakeRollerIO.stop();
                intakePivotIO.setPivotAngle(homeAngle);
                break;
            case DEPLOY_INTAKE_HOLDING:
                intakeRollerIO.setVoltage(intakeVoltage);
                intakePivotIO.setPivotAngle(deployAngle);
                break;
            case OFF:
                break;
        }

        if (consts.TUNING) {
            deployAngle = ELEVATED_POSITION.get();
            shootVoltage = ROLLER_VOLTAGE.get();
            homeAngle = HOME_POSITION.get();
            intakeVoltage = INTAKE_VOLTAGE.get();
            rollerAmpsHasCoral = HAS_CORAL_CURRENT_THRESHOLD.get();
            intakeTime = 1.0;
            shootAngle = EJECT_POSITION.get();
            intakeHoldVoltage = INTAKE_HOLD_VOLTAGE.get();
        }
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case DEPLOY_WITHOUT_ROLL -> SystemState.DEPLOY_WITHOUT_ROLLING;
            case DEPLOY_INTAKE -> {
                if (lowerAngle) {
                    yield SystemState.TREMBLE_INTAKING;
                } else {
                    yield SystemState.DEPLOY_INTAKING;
                }
            }
            case DEPLOY_INTAKE_HOLD -> SystemState.DEPLOY_INTAKE_HOLDING;
            case TREMBLE_INTAKE -> SystemState.TREMBLE_INTAKING;
            case OUTTAKE -> SystemState.OUTTAKING;
            case HOLD_OUTTAKE -> SystemState.HOLD_OUTTAKING;
            case HOME -> SystemState.HOMING;
            case SHOOT -> SystemState.SHOOTING;
            case DEPLOY_SHOOT -> SystemState.DEPLOY_SHOOTING;
            case GROUNDZERO -> SystemState.GROUNDZEROING;
            case OFF -> SystemState.OFF;
        };
    }
/*
    public void trembleIntake() {
        intakeRollerIO.setVoltage(intakeVoltage);
        intakePivotIO.setPivotAngle(deployAngle + 2);
    }

    public void zeroIntakeGround() {
        intakeRollerIO.stop();
        if (!isNearAngle(101) && !hasHomed) {
            intakePivotIO.setPivotAngle(100);
            return;
        }
        hasHomed = true;
        if (RobotBase.isReal()) {
            if (currentFilterValue <= 18) {
                intakePivotIO.setMotorVoltage(0.5);
                setWantedState(WantedState.GROUNDZERO);
            }
            if (currentFilterValue > 18) {
                intakePivotIO.setMotorVoltage(0);
                intakePivotIO.resetAngle(120);
                setWantedState(WantedState.HOME);
                hasHomed = false;
            }
        } else {
            intakePivotIO.setPivotAngle(0);
            setWantedState(WantedState.HOME);
            hasHomed = false;
        }
    }

    private void rollerIntake() {
        if (inputs.statorCurrentAmps > rollerAmpsHasCoral && !timerStarted) {
            timer.start();
            timerStarted = true;
        }
        if (inputs.statorCurrentAmps < rollerAmpsHasCoral && timerStarted && !shouldOuttake) {
            timer.stop();
            timer.reset();
            timerStarted = false;
        }
        if (timerStarted && timer.hasElapsed(outtakeTime)) {
            intakeRollerIO.setVoltage(outtakeVoltage);
            shouldOuttake = true;
            if (timer.hasElapsed(intakeTime)) {
                intakeRollerIO.setVoltage(intakeVoltage);
                timer.stop();
                timer.reset();
                shouldOuttake = false;
                timerStarted = false;
            }
        } else {
            intakeRollerIO.setVoltage(intakeVoltage);
        }

    }*/
    
    /*
    private void rollerHoldIntake() {
        if (inputs.statorCurrentAmps > rollerAmpsHasCoral && !timerStarted) {
            timer.start();
            timerStarted = true;
        }
        if (inputs.statorCurrentAmps < rollerAmpsHasCoral && timerStarted && !shouldOuttake) {
            timer.stop();
            timer.reset();
            timerStarted = false;
        }
        if (timerStarted && timer.hasElapsed(outtakeTime)) {
            intakeRollerIO.setVoltage(outtakeVoltage);
            shouldOuttake = true;
            if (timer.hasElapsed(intakeTime)) {
                intakeRollerIO.setVoltage(intakeHoldVoltage);
                timer.stop();
                timer.reset();
                shouldOuttake = false;
                timerStarted = false;
            }
        } else {
            intakeRollerIO.setVoltage(intakeHoldVoltage);
        }

    } */


    public boolean isNearAngle(double targetAngleDeg) {
        return MathUtil.isNear(targetAngleDeg, intakePivotIOInputs.currentAngleDeg, 1);
        //TODO tolerance ++
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public enum WantedState {
        DEPLOY_WITHOUT_ROLL,
        DEPLOY_INTAKE,
        TREMBLE_INTAKE,
        OUTTAKE,
        HOLD_OUTTAKE,
        HOME,
        GROUNDZERO,
        DEPLOY_SHOOT,
        SHOOT,
        DEPLOY_INTAKE_HOLD,
        OFF,
    }

    public enum SystemState {
        DEPLOY_WITHOUT_ROLLING,
        DEPLOY_INTAKING,
        TREMBLE_INTAKING,
        OUTTAKING,
        HOLD_OUTTAKING,
        HOMING,
        GROUNDZEROING,
        DEPLOY_SHOOTING,
        SHOOTING,
        DEPLOY_INTAKE_HOLDING,
        OFF,
    }
}