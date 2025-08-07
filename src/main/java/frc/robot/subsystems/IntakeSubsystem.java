package frc.robot.subsystems;

import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.INTAKE_HOME_POSITION;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;

public class IntakeSubsystem extends SubsystemBase {

    // Motors
    private final TalonFX pivotMotor = new TalonFX(Constants.CANID.PIVOTMOTOR);
    private final TalonFX rollerMotor = new TalonFX(Constants.CANID.ROLLERMOTOR);
    private final CANcoder pivotEncoder = new CANcoder(Constants.CANID.PIVOTENCODER);

    // Controls
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    // private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    // State variables
    private double zeroPosition = 0.0
            ; // Current zero position (set by Y button)
    private double targetPosition = 0.0; // Current target position for position control
    private boolean isIntaking = false; // Whether the roller is currently intaking
    private boolean isEjecting = false; // Whether the roller is currently ejecting
    private boolean isHolding = false; // Whether the pivot is in hold position

    public IntakeSubsystem() {
        configureMotors();
        CANcoderConfiguration CANcoder = new CANcoderConfiguration();
        CANcoder.MagnetSensor.MagnetOffset = -0.230712890625; // Set your desired offset here
        CANcoder.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        pivotEncoder.getConfigurator().apply(CANcoder);

    }
    public void configureMotors() {
        // Configure pivot motor
        Slot0Configs pivotSlot0Configs = new Slot0Configs();
        pivotSlot0Configs.kP = Constants.PID.pivotPID.kP.get();
        pivotSlot0Configs.kI = Constants.PID.pivotPID.kI.get();
        pivotSlot0Configs.kD = Constants.PID.pivotPID.kD.get();

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        pivotConfig.Feedback.FeedbackRemoteSensorID = 8;
        pivotConfig.Feedback.RotorToSensorRatio = 12.0;
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.Slot0 = pivotSlot0Configs;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = Constants.Limits.Intake.PIVOT_SUPPLY_CURRENT_LIMIT;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.StatorCurrentLimit = Constants.Limits.Intake.PIVOT_STATOR_CURRENT_LIMIT;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Configure roller motor
        Slot0Configs rollerSlot0Configs = new Slot0Configs();
        rollerSlot0Configs.kP = Constants.PID.rollerPID.kP.get();
        rollerSlot0Configs.kI = Constants.PID.rollerPID.kI.get();
        rollerSlot0Configs.kD = Constants.PID.rollerPID.kD.get();

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerConfig.Slot0 = rollerSlot0Configs;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = Constants.Limits.Intake.ROLLER_SUPPLY_CURRENT_LIMIT;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.StatorCurrentLimit = Constants.Limits.Intake.ROLLER_STATOR_CURRENT_LIMIT;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Apply configurations
        var pivotResult = pivotMotor.getConfigurator().apply(pivotConfig);
        var rollerResult = rollerMotor.getConfigurator().apply(rollerConfig);
    }



    /**
     * Move pivot to zero position (called by B button)
     */
    public void goToZero() {
        targetPosition = INTAKE_HOME_POSITION.get();
        isHolding = false; // Reset holding state when going to zero
    }

    /**
     * Start roller eject (clockwise) - called by LT button
     */
    public void startEject() {
        double power = Constants.INTAKE_ROLLER_POWER.get();
        rollerMotor.setControl(dutyCycleRequest.withOutput(power));
        isEjecting = true;
        isIntaking = false;
    }

    /**
     * Stop roller eject
     */
    public void stopEject() {
        rollerMotor.setControl(dutyCycleRequest.withOutput(0.0));
        isEjecting = false;
        System.out.println("IntakeSubsystem: Eject stopped");
    }

    /**
     * Start roller intake (counterclockwise) - called by RB button
     */
    public void startIntake() {
        System.out.println("IntakeSubsystem: startIntake() called");
        double power = -Constants.INTAKE_ROLLER_POWER.get()-0.4;
        System.out.println("IntakeSubsystem: Starting intake with power: " + power);
        rollerMotor.setControl(dutyCycleRequest.withOutput(power));
        isIntaking = true;
        isEjecting = false;
        System.out.println("IntakeSubsystem: Intake started successfully");
    }

    /**
     * Stop roller intake
     */
    public void stopIntake() {
        System.out.println("IntakeSubsystem: stopIntake() called");
        rollerMotor.setControl(dutyCycleRequest.withOutput(0.0));
        isIntaking = false;
        System.out.println("IntakeSubsystem: Intake stopped");
    }

    /**
     * Stop roller movement (stops both intake and eject)
     */
    public void stopRoller() {
        System.out.println("IntakeSubsystem: stopRoller() called");
        rollerMotor.setControl(dutyCycleRequest.withOutput(0.0));
        isIntaking = false;
        isEjecting = false;
        System.out.println("IntakeSubsystem: Roller stopped");
    }

    /**
     * Move pivot counterclockwise to elevated position (called by LB button)
     */
    public void elevate() {
        System.out.println("IntakeSubsystem: elevate() called");
        // Move pivot counterclockwise to elevated position (negative value = counterclockwise)
        targetPosition = zeroPosition - Constants.INTAKE_ELEVATED_POSITION.get();
        System.out.println("IntakeSubsystem: Moving counterclockwise to elevated position: " + targetPosition);
        isHolding = true;
        System.out.println("IntakeSubsystem: Counterclockwise elevated target set - position control active");
    }

    /**
     * Check if pivot is at zero position
     */
    public boolean isAtZero() {
        double currentPosition = pivotMotor.getPosition().getValueAsDouble();
        double homePosition = zeroPosition + INTAKE_HOME_POSITION.get();
        double positionError = Math.abs(currentPosition - homePosition);
        return positionError < Constants.INTAKE_POSITION_TOLERANCE.get();
    }

    /**
     * Check if pivot is at elevated position
     */
    public boolean isAtElevated() {
        double currentPosition = pivotMotor.getPosition().getValueAsDouble();
        double elevatedPosition = zeroPosition + Constants.INTAKE_ELEVATED_POSITION.get();
        double positionError = Math.abs(currentPosition - elevatedPosition);
        return positionError < Constants.INTAKE_POSITION_TOLERANCE.get();
    }

    /**
     * Check if pivot is at target position
     */
    public boolean isAtTarget() {
        double currentPosition = pivotMotor.getPosition().getValueAsDouble();
        double positionError = Math.abs(currentPosition - targetPosition);
        return positionError < Constants.INTAKE_POSITION_TOLERANCE.get();
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

    /**
     * Returns whether the roller is currently intaking
     */
    public boolean isIntaking() {
        return isIntaking;
    }

    public void goToEjectPosition() {
        System.out.println("IntakeSubsystem: goToEjectPosition() called");
        // Move pivot counterclockwise to eject position (negative value = counterclockwise)
        targetPosition = zeroPosition - Constants.INTAKE_EJECT_POSITION.get();
        isHolding = false;
        System.out.println("IntakeSubsystem: Moving counterclockwise to eject position: " + targetPosition);
        System.out.println("IntakeSubsystem: Eject target set - position control active");
    }
    // Removed getAutonomousCommand() from IntakeSubsystem. Autonomous logic belongs in RobotContainer.
    private void updatePID() {
        // Check if any PID values have changed
        boolean pivotPIDChanged = Constants.PID.pivotPID.kP.hasChanged() || Constants.PID.pivotPID.kI.hasChanged() || Constants.PID.pivotPID.kD.hasChanged();
        boolean rollerPIDChanged = Constants.PID.rollerPID.kP.hasChanged() || Constants.PID.rollerPID.kI.hasChanged() || Constants.PID.rollerPID.kD.hasChanged();

        // If any values changed, reconfigure the motors
        if (pivotPIDChanged || rollerPIDChanged) {
            Logger.recordOutput("Intake/PID/Reconfiguring", true);
            configureMotors();
            Logger.recordOutput("Intake/PID/Pivot_kP", Constants.PID.pivotPID.kP.get());
            Logger.recordOutput("Intake/PID/Pivot_kI", Constants.PID.pivotPID.kI.get());
            Logger.recordOutput("Intake/PID/Pivot_kD", Constants.PID.pivotPID.kD.get());
            Logger.recordOutput("Intake/PID/Roller_kP", Constants.PID.rollerPID.kP.get());
            Logger.recordOutput("Intake/PID/Roller_kI", Constants.PID.rollerPID.kI.get());
            Logger.recordOutput("Intake/PID/Roller_kD", Constants.PID.rollerPID.kD.get());
        } else {
            Logger.recordOutput("Intake/PID/Reconfiguring", false);
        }
    }

    private void updateTargetPosition() {
        // Check if position constants have changed
        boolean homePositionChanged = INTAKE_HOME_POSITION.hasChanged();
        boolean elevatedPositionChanged = Constants.INTAKE_ELEVATED_POSITION.hasChanged();

        if (homePositionChanged || elevatedPositionChanged) {
            System.out.println("IntakeSubsystem: Position constants changed - updating target");
            
            if (isHolding) {
                // If currently holding (LB position), update to new elevated position (counterclockwise)
                targetPosition = zeroPosition + Constants.INTAKE_ELEVATED_POSITION.get();
                System.out.println("IntakeSubsystem: Updated LB counterclockwise target to: " + targetPosition);
            } else {
                // If not holding, update to new home position
                targetPosition = zeroPosition - INTAKE_HOME_POSITION.get();
                System.out.println("IntakeSubsystem: Updated home target to: " + targetPosition);
            }
        }
    }
    public void log() {
        // Pivot motor logging
        Logger.recordOutput("Intake/Pivot/Position", pivotMotor.getPosition().getValue());
        Logger.recordOutput("Intake/Pivot/Velocity", pivotMotor.getVelocity().getValue());
        Logger.recordOutput("Intake/Pivot/Current", pivotMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Intake/Pivot/Voltage", pivotMotor.getSupplyVoltage().getValue());
        Logger.recordOutput("Intake/Pivot/ZeroPosition", zeroPosition);
        Logger.recordOutput("Intake/Pivot/TargetPosition", targetPosition);
        Logger.recordOutput("Intake/Pivot/IsAtZero", isAtZero());
        Logger.recordOutput("Intake/Pivot/IsAtElevated", isAtElevated());
        Logger.recordOutput("Intake/Pivot/IsAtTarget", isAtTarget());
        Logger.recordOutput("Intake/Pivot/IsHolding", isHolding);
        Logger.recordOutput("Intake/Pivot/CANcoderPosition", pivotEncoder.getAbsolutePosition().getValue());

        // Roller motor logging
        Logger.recordOutput("Intake/Roller/Velocity", rollerMotor.getVelocity().getValue());
        Logger.recordOutput("Intake/Roller/StatorCurrent", rollerMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Intake/Roller/SupplyCurrent", rollerMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Intake/Roller/Voltage", rollerMotor.getSupplyVoltage().getValue());
        Logger.recordOutput("Intake/Roller/State", getRollerState());
        Logger.recordOutput("Intake/Roller/IsIntaking", isIntaking);
        Logger.recordOutput("Intake/Roller/IsEjecting", isEjecting);

        // Tunable constants logging
        Logger.recordOutput("Intake/Constants/RollerSpeed", Constants.INTAKE_ROLLER_POWER.get());
        Logger.recordOutput("Intake/Constants/ElevatedPosition", Constants.INTAKE_ELEVATED_POSITION.get());
        Logger.recordOutput("Intake/Constants/PositionTolerance", Constants.INTAKE_POSITION_TOLERANCE.get());
        Logger.recordOutput("Intake/Constants/HomePosition", INTAKE_HOME_POSITION.get());
        Logger.recordOutput("Intake/Constants/PivotSpeed", Constants.INTAKE_PIVOT_SPEED.get());
        
        // Enhanced pivot position logging
        Logger.recordOutput("Intake/Pivot/CurrentPosition", pivotMotor.getPosition().getValue());
        Logger.recordOutput("Intake/Pivot/ZeroPosition", zeroPosition);
        Logger.recordOutput("Intake/Pivot/TargetPosition", targetPosition);
        Logger.recordOutput("Intake/Pivot/PositionError", Math.abs(pivotMotor.getPosition().getValueAsDouble() - targetPosition));
    }

    @Override
    public void periodic() {
        updatePID();
        updateTargetPosition();
        
        // Apply continuous position control to pivot motor
        pivotMotor.setControl(positionRequest.withPosition(targetPosition));
        
        log();
        
        // Debug: Show motor data
        double pivotPosition = pivotMotor.getPosition().getValueAsDouble();
        double rollerVelocity = rollerMotor.getVelocity().getValueAsDouble();
        double rollerCurrent = rollerMotor.getStatorCurrent().getValueAsDouble();
    }
}
