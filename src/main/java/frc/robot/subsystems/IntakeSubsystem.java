package frc.robot.subsystems;

import java.util.Vector;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.consts;
import frc.robot.consts.Limits.Intake;

// TODO: Auto Intake Auto Eject based on current

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
    private final LinearFilter currentFilter = LinearFilter.movingAverage(5); // Filter for coral detection
    private final Timer timer = new Timer();
    
    // State variables
    
    private double zeroPosition = 0.0; // Current zero position (set by Y button)
    public static enum CurrentState {INTAKING, EJECTING, HOLDING, ELEVATING, ZEROING, IDLE}
    public static enum WantedState {INTAKE, EJECT, HOLD, ELEVATE, ZERO}
    public static enum IntakePosition {ZERO, HOLD, ELEVATE}
    private CurrentState currentState = CurrentState.IDLE; // Current state of the intake subsystem
    private WantedState wantedState = WantedState.ZERO; // Desired state of the intake subsystem
    private IntakePosition intakePosition = IntakePosition.ZERO; // Current position of the intake subsystem
    private Vector<WantedState> scheduledTasks = new Vector<WantedState>(); // Scheduled tasks for the intake subsystem
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
        
        // System.out.println("IntakeSubsystem: Pivot motor config result: " + pivotResult);
        // System.out.println("IntakeSubsystem: Roller motor config result: " + rollerResult);
        // System.out.println("IntakeSubsystem: Pivot motor CAN ID: " + consts.CANID.MOTOR_PIVOT);
        // System.out.println("IntakeSubsystem: Roller motor CAN ID: " + consts.CANID.MOTOR_ROLLER);
        // System.out.println("IntakeSubsystem: Roller PID - kP: " + consts.PID.rollerPID.kP.get() + ", kI: " + consts.PID.rollerPID.kI.get() + ", kD: " + consts.PID.rollerPID.kD.get());
    }

    /**
     * Detect if the roller has coral based on velocity and current
     * This method only works when motorRoller has a voltage applied
     * @return true if coral is detected, false otherwise
     */
    public boolean detectCoral() {
        // If the roller does not have a voltage applied, it is impossible to hold a coral so we short circuit this check
        if (motorRoller.getSupplyVoltage().getValueAsDouble() < consts.INTAKE_ROLLER_VOLTAGE.get() / 2) {
            return false;
        }

        // Check if the roller is moving and if the filter detects a high current
        double rollerVelocity = motorRoller.getVelocity().getValueAsDouble();
        double rollerCurrent = currentFilter.calculate(motorRoller.getStatorCurrent().getValueAsDouble());

        if (rollerVelocity < consts.Superstructures.Intake.HAS_CORAL_VELOCITY_THRESHOLD.get() && 
            rollerCurrent > consts.Superstructures.Intake.HAS_CORAL_CURRENT_THRESHOLD.get()) {
            return true;
        }

        return false;
    }

    /**
     * Move pivot to zero position (called by B button)
     */
    public void goToZero() {
        motorPivot.setControl(positionRequest.withPosition(zeroPosition));
    }

    /**
     * Start roller intake (counterclockwise) - called by RB button
     */
    public void intake() {
        motorRoller.setControl(dutyCycleRequest.withOutput(-consts.INTAKE_ROLLER_POWER.get()));        
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
        motorRoller.setControl(dutyCycleRequest.withOutput(consts.INTAKE_ROLLER_POWER.get()));
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
        motorPivot.setControl(positionRequest.withPosition(zeroPosition - consts.INTAKE_ELEVATED_POSITION.get()));
    }

    /**
     * Move pivot counterclockwise to eject position
     */
    public void goToEjectPosition() {
        motorPivot.setControl(positionRequest.withPosition(zeroPosition - consts.INTAKE_EJECT_POSITION.get()));
    }

    /**
     * Check if pivot is at zero position
     */
    public boolean isAtZero() {
        double currentPosition = motorPivot.getPosition().getValueAsDouble();
        double homePosition = zeroPosition + consts.INTAKE_HOME_POSITION.get();
        double positionError = Math.abs(currentPosition - homePosition);
        return positionError < consts.INTAKE_POSITION_TOLERANCE.get();
    }

    /**
     * Check if pivot is at holding position
     */
    public boolean isAtHoldingPosition() {
        double currentPosition = motorPivot.getPosition().getValueAsDouble();
        double holdingPosition = zeroPosition + consts.INTAKE_HOME_POSITION.get() + consts.INTAKE_HOLD_OFFSET.get();
        double positionError = Math.abs(currentPosition - holdingPosition);
        return positionError < consts.INTAKE_POSITION_TOLERANCE.get();
    }

    /**
     * Check if pivot is at elevated position
     */
    public boolean isAtElevated() {
        double currentPosition = motorPivot.getPosition().getValueAsDouble();
        double elevatedPosition = zeroPosition + consts.INTAKE_ELEVATED_POSITION.get();
        double positionError = Math.abs(currentPosition - elevatedPosition);
        return positionError < consts.INTAKE_POSITION_TOLERANCE.get();
    }

    public void setWantedState(WantedState state) {
        scheduledTasks.add(state); // Add the state to the scheduled tasks
    }

    public void processWantedState() {
        // If the current state is not IDLE, we short circuit.
        if (currentState != CurrentState.IDLE) {
            return;
        }

        wantedState = scheduledTasks.remove(0); // Get the first scheduled task

        switch (wantedState) {
            case INTAKE:
                currentState = CurrentState.INTAKING;
                break;
            case EJECT:
                currentState = CurrentState.EJECTING;
                timer.reset();
                timer.start();
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
                    wantedState = WantedState.HOLD; // After stopping intake, we want to hold the position
                } else {
                    // Continue intake until a coral is detected
                    intake();
                }
                break;
            case EJECTING:
                // If the eject timer has expired, stop the eject
                if (timer.hasElapsed(consts.Superstructures.Intake.EJECT_TIME.get())) {
                    stopEject();
                    currentState = CurrentState.IDLE; // No scheduled task, so we go to default state
                    wantedState = WantedState.ZERO; // After ejecting, we want to go to zero position
                } else {
                    // Continue ejecting until the timer expires
                    eject();
                }
                break;
            case HOLDING:
                // If we reach holding point, go IDLE
                if (isAtHoldingPosition()) {
                    currentState = CurrentState.IDLE;
                } else {
                    goToEjectPosition();
                }
                break;
            case ELEVATING:
                // If the pivot is at the elevated position, turn to IDLE
                if (isAtElevated()) {
                    currentState = CurrentState.IDLE;
                } else {
                    elevate();
                }
                break;
            case ZEROING:
                // If the pivot is at the zero position, turn to IDLE
                if (isAtZero()) {
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
        Logger.recordOutput("Intake/Pivot/ZeroPosition", zeroPosition);
        Logger.recordOutput("Intake/Pivot/IsAtZero", isAtZero());
        Logger.recordOutput("Intake/Pivot/IsAtElevated", isAtElevated());
        Logger.recordOutput("Intake/Pivot/CANcoderPosition", encoderPivot.getAbsolutePosition().getValue());

        // Roller motor logging
        Logger.recordOutput("Intake/Roller/Velocity", motorRoller.getVelocity().getValue());
        Logger.recordOutput("Intake/Roller/StatorCurrent", motorRoller.getStatorCurrent().getValue());
        Logger.recordOutput("Intake/Roller/SupplyCurrent", motorRoller.getSupplyCurrent().getValue());
        Logger.recordOutput("Intake/Roller/Voltage", motorRoller.getSupplyVoltage().getValue());

        // Tunable constants logging
        Logger.recordOutput("Intake/Constants/RollerSpeed", consts.INTAKE_ROLLER_POWER.get());
        Logger.recordOutput("Intake/Constants/HoldOffset", consts.INTAKE_HOLD_OFFSET.get());
        Logger.recordOutput("Intake/Constants/ElevatedPosition", consts.INTAKE_ELEVATED_POSITION.get());
        Logger.recordOutput("Intake/Constants/PositionTolerance", consts.INTAKE_POSITION_TOLERANCE.get());
        Logger.recordOutput("Intake/Constants/HomePosition", consts.INTAKE_HOME_POSITION.get());
        Logger.recordOutput("Intake/Constants/PivotSpeed", consts.INTAKE_PIVOT_SPEED.get());
        
        // Enhanced pivot position logging
        Logger.recordOutput("Intake/Pivot/CurrentPosition", motorPivot.getPosition().getValue());
        Logger.recordOutput("Intake/Pivot/ZeroPosition", zeroPosition);

        // Coral detection logging
        Logger.recordOutput("Intake/Coral/HasCoral", hasCoral);
    }

    @Override
    public void periodic() {
        // Coral detection logic
        hasCoral = detectCoral();

        // Process the current state of the intake subsystem
        processWantedState();
        processState();

        updatePID();

        log();
    }
}
