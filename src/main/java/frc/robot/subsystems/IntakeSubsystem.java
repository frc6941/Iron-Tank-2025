package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.Queue;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.consts;
import frc.robot.consts.Limits.Intake;

/*
 * Positions
 * ZERO:        |
 * EJECT:       /
 * ELEVATE:     __
 */
public class IntakeSubsystem extends SubsystemBase {
    // Motors
    private final TalonFX motorPivot = new TalonFX(consts.CANID.MOTOR_PIVOT);
    private final TalonFX motorRoller = new TalonFX(consts.CANID.MOTOR_ROLLER);
    private final CANcoder encoderPivot = new CANcoder(consts.CANID.ENCODER_PIVOT);

    // Controls
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final LinearFilter currentFilter = LinearFilter.movingAverage((int) consts.Superstructures.Intake.LF_WINDOW_SIZE.get()); // Filter for coral detection
    private final Timer timer = new Timer();
    
    // State variables
    public static enum CurrentState {INTAKING, EJECTING, HOLDING, ELEVATING, ZEROING, IDLE}
    public static enum WantedState {INTAKE, EJECT, HOLD, ELEVATE, ZERO}
    public static enum IntakePosition {ZERO, EJECT, ELEVATE, BUSY}
    private CurrentState currentState = CurrentState.IDLE; // Current state of the intake subsystem
    private WantedState wantedState = WantedState.ZERO; // Desired state of the intake subsystem
    private IntakePosition intakePosition = IntakePosition.ZERO; // Current position of the intake subsystem
    private boolean hasCoral = false;

    public IntakeSubsystem() {
        configureMotors();
        CANcoderConfiguration CANcoder = new CANcoderConfiguration();
        CANcoder.MagnetSensor.MagnetOffset = consts.Sensors.Encoder.INTAKE_CANCODER_MAGET_OFFSET;
        CANcoder.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderPivot.getConfigurator().apply(CANcoder);
    }

    public void configureMotors() {
        // Configure pivot motor
        Slot0Configs pivotSlot0Configs = new Slot0Configs();
        pivotSlot0Configs.kP = consts.PID.pivotPID.kP.get();
        pivotSlot0Configs.kI = consts.PID.pivotPID.kI.get();
        pivotSlot0Configs.kD = consts.PID.pivotPID.kD.get();

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        pivotConfig.Feedback.FeedbackRemoteSensorID = consts.CANID.ENCODER_PIVOT;
        pivotConfig.Feedback.RotorToSensorRatio = consts.Sensors.Encoder.ROTOR_TO_SENSOR_RATIO;
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.Slot0 = pivotSlot0Configs;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = consts.Limits.Intake.PIVOT_SUPPLY_CURRENT_LIMIT;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.StatorCurrentLimit = consts.Limits.Intake.PIVOT_STATOR_CURRENT_LIMIT;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Configure roller motor
        Slot0Configs rollerSlot0Configs = new Slot0Configs();
        rollerSlot0Configs.kP = consts.PID.rollerPID.kP.get();
        rollerSlot0Configs.kI = consts.PID.rollerPID.kI.get();
        rollerSlot0Configs.kD = consts.PID.rollerPID.kD.get();

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerConfig.Slot0 = rollerSlot0Configs;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = consts.Limits.Intake.ROLLER_SUPPLY_CURRENT_LIMIT;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.StatorCurrentLimit = consts.Limits.Intake.ROLLER_STATOR_CURRENT_LIMIT;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        motorPivot.getConfigurator().apply(pivotConfig);
        motorRoller.getConfigurator().apply(rollerConfig);
    }

    /**
     * Detect if the roller has coral based on velocity and current
     * This method only works when motorRoller has a voltage applied
     * @return true if coral is detected, false otherwise
     */
    public boolean detectCoral() {
        // Check if the roller is moving and if the filter detects a high current
        double rollerVelocity = motorRoller.getVelocity().getValueAsDouble();
        double rollerCurrent = currentFilter.calculate(motorRoller.getStatorCurrent().getValueAsDouble());

        if (rollerVelocity < consts.Superstructures.Intake.HAS_CORAL_VELOCITY_THRESHOLD.get() && 
            rollerCurrent > consts.Superstructures.Intake.HAS_CORAL_CURRENT_THRESHOLD.get()) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Move pivot to zero position (called by B button)
     */
    public void goToZero() {
        motorPivot.setControl(positionRequest.withPosition(0));
    }

    /**
     * Start roller intake (counterclockwise) - called by RB button
     */
    public void intake() {
        motorRoller.setControl(dutyCycleRequest.withOutput(-consts.Superstructures.Intake.ROLLER_POWER.get()));        
    }

    /**
     * Stop roller intake
     */
    public void stopIntake() {
        motorRoller.setControl(dutyCycleRequest.withOutput(0.0));
    }

    /**
     * Start roller eject (clockwise) - called by LT button
     */
    public void eject() {
        motorRoller.setControl(dutyCycleRequest.withOutput(consts.Superstructures.Intake.ROLLER_POWER.get()));
    }

    /**
     * Stop roller eject
     */
    public void stopEject() {
        motorRoller.setControl(dutyCycleRequest.withOutput(0.0));
    }

    /**
     * Move pivot counterclockwise to elevated position (called by LB button)
     */
    public void elevate() {
        // Move pivot counterclockwise to elevated position (negative value = counterclockwise)
        motorPivot.setControl(positionRequest.withPosition(-consts.Superstructures.Intake.ELEVATED_POSITION.get()));
    }

    /**
     * Holds the coral
     */
    public void hold(){
        motorRoller.setControl(voltageRequest.withOutput(-consts.Superstructures.Intake.HOLD_VOLTAGE.get()));
    }

    /**
     * Move pivot counterclockwise to eject position
     */
    public void goToEjectPosition() {
        motorPivot.setControl(positionRequest.withPosition(-consts.Superstructures.Intake.EJECT_POSITION.get()));
    }

    /**
     * Check if pivot is at zero position
     */
    public boolean isAtZero() {
        double currentPosition = motorPivot.getPosition().getValueAsDouble();
        double positionError = Math.abs(currentPosition);
        return positionError < consts.Superstructures.Intake.POSITION_TOLERANCE.get();
    }

    /**
     * Check if pivot is at holding position
     */
    public boolean isAtEjectingPosition() {
        double currentPosition = motorPivot.getPosition().getValueAsDouble();
        double holdingPosition = -consts.Superstructures.Intake.EJECT_POSITION.get();
        double positionError = Math.abs(currentPosition - holdingPosition);
        return positionError < consts.Superstructures.Intake.POSITION_TOLERANCE.get();
    }

    /**
     * Check if pivot is at elevated position
     */
    public boolean isAtElevated() {
        double currentPosition = motorPivot.getPosition().getValueAsDouble();
        double elevatedPosition = -consts.Superstructures.Intake.ELEVATED_POSITION.get();
        double positionError = Math.abs(currentPosition - elevatedPosition);
        return positionError < consts.Superstructures.Intake.POSITION_TOLERANCE.get();
    }

    public void checkPosition() {
        if (isAtZero()) {
            intakePosition = IntakePosition.ZERO;
        } else if (isAtEjectingPosition()) {
            intakePosition = IntakePosition.EJECT;
        } else if (isAtElevated()) {
            intakePosition = IntakePosition.ELEVATE;
        } else {
            intakePosition = IntakePosition.BUSY;
        }
    }

    public boolean isShootFinished() {
        // A "shoot" is finished if the intake is no longer in the EJECTING state
        return currentState != CurrentState.EJECTING;
    }

    public void setWantedState(WantedState state) {
        wantedState = state;
    }

    public void processWantedState() {
        // If the current state is not IDLE or HOLDING, we are busy so we short circuit.
        if (currentState != CurrentState.IDLE && currentState != CurrentState.HOLDING) {
            return;
        }
        
        switch (wantedState) {
            case INTAKE:
                if (intakePosition == IntakePosition.ZERO || intakePosition == IntakePosition.EJECT){
                    setWantedState(WantedState.ELEVATE);
                    setWantedState(WantedState.INTAKE);
                    break;
                }
                currentState = CurrentState.INTAKING;
                break;
            case EJECT:
                currentState = CurrentState.EJECTING;
                break;
            case HOLD:
                currentState = CurrentState.HOLDING;
                break;
            case ELEVATE:
                currentState = CurrentState.ELEVATING;
                break;
            case ZERO:
                currentState = CurrentState.ZEROING;
                break;
            default:
                // We shouldn't be here, so raise a warning
                System.out.println("[WARNING] IntakeSubsystem: Invalid wanted state: " + wantedState);
                break;
        }
    }

    public void processState() {
        // Process the current state of the intake subsystem
        switch (currentState) {
            case INTAKING:
                // If we have a coral detected, stop the intake
                if (hasCoral) {
                    stopIntake();
                    currentState = CurrentState.IDLE; // Go IDLE state when stopping intake
                    wantedState = WantedState.HOLD; // After stopping intake, we want to goto HOLD
                } else {
                    // Continue intake until a coral is detected
                    intake();
                }
                break;
            case EJECTING:
                // Go to eject position
                if (intakePosition != IntakePosition.EJECT) {
                    hold();
                    goToEjectPosition();
                    break;
                }
                // This part runs only when in position. Start the timer once.
                if (!timer.isRunning()) {
                    timer.reset();
                    timer.start();
                }
                // If the eject timer has expired, stop the eject
                if (timer.hasElapsed(consts.Superstructures.Intake.EJECT_TIME.get())) {
                    stopEject();
                    timer.stop();
                    hasCoral = false; // Reset coral detection after ejecting
                    currentState = CurrentState.IDLE; // No scheduled task, so we go to default state
                    wantedState = WantedState.ZERO; // After ejecting, we want to go to zero position
                } else {
                    // Continue ejecting until the timer expires
                    eject();
                }
                break;
            case HOLDING:
                // Holds coral regardless of position
                hold();
                // If we reach holding point, go HOLDING
                if (intakePosition != IntakePosition.ZERO) {
                    goToZero();
                }
                break;
            case ELEVATING:
                // If the pivot is at the elevated position, turn to IDLE
                if (intakePosition == IntakePosition.ELEVATE) {
                    currentState = CurrentState.IDLE;
                } else {
                    elevate();
                }
                break;
            case ZEROING:
                // If the pivot is at the zero position, turn to IDLE
                if (intakePosition == IntakePosition.ZERO) {
                    currentState = CurrentState.IDLE;
                } else {
                    goToZero();
                }
                break;
            default:
                // IDLE means nothing to do.
                break;
        }
    }

    private void updatePID() {
        // Check if any PID values have changed
        boolean pivotPIDChanged = consts.PID.pivotPID.kP.hasChanged() || consts.PID.pivotPID.kI.hasChanged() || consts.PID.pivotPID.kD.hasChanged();
        boolean rollerPIDChanged = consts.PID.rollerPID.kP.hasChanged() || consts.PID.rollerPID.kI.hasChanged() || consts.PID.rollerPID.kD.hasChanged();

        // If any values changed, reconfigure the motors
        if (pivotPIDChanged || rollerPIDChanged) {
            configureMotors();
        }
    }

    public void log() {
        // Pivot motor logging
        Logger.recordOutput("Intake/Pivot/Position", motorPivot.getPosition().getValue());
        Logger.recordOutput("Intake/Pivot/Velocity", motorPivot.getVelocity().getValue());
        Logger.recordOutput("Intake/Pivot/Current", motorPivot.getStatorCurrent().getValue());
        Logger.recordOutput("Intake/Pivot/Voltage", motorPivot.getSupplyVoltage().getValue());
        Logger.recordOutput("Intake/Pivot/position", intakePosition);
        Logger.recordOutput("Intake/Pivot/CANcoderPosition", encoderPivot.getAbsolutePosition().getValue());

        // Roller motor logging
        Logger.recordOutput("Intake/Roller/Velocity", motorRoller.getVelocity().getValue());
        Logger.recordOutput("Intake/Roller/StatorCurrent", motorRoller.getStatorCurrent().getValue());
        Logger.recordOutput("Intake/Roller/SupplyCurrent", motorRoller.getSupplyCurrent().getValue());
        Logger.recordOutput("Intake/Roller/Voltage", motorRoller.getSupplyVoltage().getValue());
        
        // Enhanced pivot position logging
        Logger.recordOutput("Intake/Pivot/CurrentPosition", motorPivot.getPosition().getValue());

        // Coral detection logging
        Logger.recordOutput("Intake/Coral/HasCoral", hasCoral);

        // States
        Logger.recordOutput("Intake/State/Position", intakePosition);
        Logger.recordOutput("Intake/State/WantedState", wantedState);
        Logger.recordOutput("Intake/State/CurrentState", currentState);
    }

    @Override
    public void periodic() {
        hasCoral = detectCoral();

        // Process the current state of the intake subsystem
        checkPosition();
        processWantedState();
        processState();

        updatePID();

        log();
    }
}
