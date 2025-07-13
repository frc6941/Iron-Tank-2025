package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.consts;

public class IntakeSubsystem extends SubsystemBase {

    // Motors
    private final TalonFX pivotMotor = new TalonFX(consts.CANID.PIVOTMOTOR);
    private final TalonFX rollerMotor = new TalonFX(consts.CANID.ROLLERMOTOR);

    // Controls
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);



    // State variables
    private double zeroPosition = 0.0; // Current zero position (set by Y button)
    private boolean isIntaking = false; // Whether the roller is currently intaking
    private boolean isEjecting = false; // Whether the roller is currently ejecting
    private boolean isHolding = false; // Whether the pivot is in hold position

    public IntakeSubsystem() {
        configureMotors();
    }

    public void configureMotors() {
        // Configure pivot motor
        Slot0Configs pivotSlot0Configs = new Slot0Configs();
        pivotSlot0Configs.kP = consts.PID.pivotPID.kP.get();
        pivotSlot0Configs.kI = consts.PID.pivotPID.kI.get();
        pivotSlot0Configs.kD = consts.PID.pivotPID.kD.get();

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.Slot0 = pivotSlot0Configs;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = consts.Limits.Intake.PIVOT_SUPPLY_CURRENT_LIMIT;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.StatorCurrentLimit = consts.Limits.Intake.PIVOT_STATOR_CURRENT_LIMIT;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Configure roller motor
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = consts.Limits.Intake.ROLLER_SUPPLY_CURRENT_LIMIT;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.StatorCurrentLimit = consts.Limits.Intake.ROLLER_STATOR_CURRENT_LIMIT;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Apply configurations
        pivotMotor.getConfigurator().apply(pivotConfig);
        rollerMotor.getConfigurator().apply(rollerConfig);
    }

    /**
     * Set current position as zero (called by Y button)
     */
    public void setZeroPosition() {
        zeroPosition = pivotMotor.getPosition().getValueAsDouble();
    }

    /**
     * Move pivot to zero position (called by first B press - Home)
     */
    public void goToHome() {
        double targetPosition = zeroPosition;
        pivotMotor.setControl(positionRequest.withPosition(targetPosition));
        isHolding = false; // Reset holding state when going to home
    }

    /**
     * Start roller intake (clockwise) - called by second B press
     */
    public void startIntake() {
        if (!isIntaking && !isEjecting) {
            rollerMotor.setControl(velocityRequest.withVelocity(consts.INTAKE_ROLLER_SPEED.get()));
            isIntaking = true;
            isEjecting = false;
        }
    }

    /**
     * Stop roller intake
     */
    public void stopIntake() {
        rollerMotor.setControl(dutyCycleRequest.withOutput(0.0));
        isIntaking = false;
    }

    /**
     * Start roller eject (counterclockwise) - called by first right shoulder press
     */
    public void startEject() {
        if (!isEjecting && !isIntaking) {
            rollerMotor.setControl(velocityRequest.withVelocity(-consts.INTAKE_ROLLER_SPEED.get()));
            isEjecting = true;
            isIntaking = false;
        }
    }

    /**
     * Stop roller eject
     */
    public void stopEject() {
        rollerMotor.setControl(dutyCycleRequest.withOutput(0.0));
        isEjecting = false;
    }

    /**
     * Stop roller movement (Hold action)
     */
    public void hold() {
        stopIntake();
        stopEject();
        // Note: isHolding will be set to true by elevate() method
    }

    /**
     * Move pivot upward (Elevate action) - pivot turns clockwise
     */
    public void elevate() {
        // Move pivot to elevated position (clockwise = upward for intake)
        double elevatedPosition = zeroPosition + consts.INTAKE_ELEVATED_POSITION.get();
        pivotMotor.setControl(positionRequest.withPosition(elevatedPosition));
        isHolding = true;
    }

    /**
     * Check if pivot is at zero position
     */
    public boolean isAtZero() {
        double currentPosition = pivotMotor.getPosition().getValueAsDouble();
        double positionError = Math.abs(currentPosition - zeroPosition);
        return positionError < consts.INTAKE_POSITION_TOLERANCE.get();
    }

    /**
     * Check if pivot is at elevated position
     */
    public boolean isAtElevated() {
        double currentPosition = pivotMotor.getPosition().getValueAsDouble();
        double elevatedPosition = zeroPosition + consts.INTAKE_ELEVATED_POSITION.get();
        double positionError = Math.abs(currentPosition - elevatedPosition);
        return positionError < consts.INTAKE_POSITION_TOLERANCE.get();
    }

    /**
     * Get current roller state
     */
    public String getRollerState() {
        if (isIntaking) return "Intaking";
        if (isEjecting) return "Ejecting";
        return "Stopped";
    }

    /**
     * Check if pivot is currently holding (elevated)
     */
    public boolean isHolding() {
        return isHolding;
    }

    private void updatePID() {
        // Check if any PID values have changed
        boolean pidChanged = consts.PID.pivotPID.kP.hasChanged() || consts.PID.pivotPID.kI.hasChanged() || consts.PID.pivotPID.kD.hasChanged();

        // If any values changed, reconfigure the motors
        if (pidChanged) {
            Logger.recordOutput("Intake/PID/Reconfiguring", true);
            configureMotors();
            Logger.recordOutput("Intake/PID/kP", consts.PID.pivotPID.kP.get());
            Logger.recordOutput("Intake/PID/kI", consts.PID.pivotPID.kI.get());
            Logger.recordOutput("Intake/PID/kD", consts.PID.pivotPID.kD.get());
        } else {
            Logger.recordOutput("Intake/PID/Reconfiguring", false);
        }
    }

    public void log() {
        // Pivot motor logging
        Logger.recordOutput("Intake/Pivot/Position", pivotMotor.getPosition().getValue());
        Logger.recordOutput("Intake/Pivot/Velocity", pivotMotor.getVelocity().getValue());
        Logger.recordOutput("Intake/Pivot/Current", pivotMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Intake/Pivot/Voltage", pivotMotor.getSupplyVoltage().getValue());
        Logger.recordOutput("Intake/Pivot/ZeroPosition", zeroPosition);
        Logger.recordOutput("Intake/Pivot/IsAtZero", isAtZero());
        Logger.recordOutput("Intake/Pivot/IsAtElevated", isAtElevated());
        Logger.recordOutput("Intake/Pivot/IsHolding", isHolding);

        // Roller motor logging
        Logger.recordOutput("Intake/Roller/Velocity", rollerMotor.getVelocity().getValue());
        Logger.recordOutput("Intake/Roller/StatorCurrent", rollerMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Intake/Roller/SupplyCurrent", rollerMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Intake/Roller/Voltage", rollerMotor.getSupplyVoltage().getValue());
        Logger.recordOutput("Intake/Roller/State", getRollerState());
        Logger.recordOutput("Intake/Roller/IsIntaking", isIntaking);
        Logger.recordOutput("Intake/Roller/IsEjecting", isEjecting);

        // Tunable constants logging
        Logger.recordOutput("Intake/Constants/RollerSpeed", consts.INTAKE_ROLLER_SPEED.get());
        Logger.recordOutput("Intake/Constants/HoldOffset", consts.INTAKE_HOLD_OFFSET.get());
        Logger.recordOutput("Intake/Constants/ElevatedPosition", consts.INTAKE_ELEVATED_POSITION.get());
        Logger.recordOutput("Intake/Constants/PositionTolerance", consts.INTAKE_POSITION_TOLERANCE.get());
    }

    @Override
    public void periodic() {
        updatePID();
        log();
    }
}
