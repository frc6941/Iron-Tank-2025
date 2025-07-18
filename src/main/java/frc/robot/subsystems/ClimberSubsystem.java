package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

// import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climberMotor = new TalonFX(Constants.CANID.MOTOR_CLIMBER);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final Timer climbTimer = new Timer();
    private boolean atStartPosition = false;
    private boolean climbing = false;

    public ClimberSubsystem() {
        configureMotor();
    }

    public void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Counterclockwise is positive
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberMotor.getConfigurator().apply(config);
    }

    /** Set climber voltage (tunable) */
    public void setClimberVoltage() {
        double voltage = Constants.CLIMBER_VOLTAGE.get();
        System.out.println("ClimberSubsystem: setClimberVoltage() called, voltage: " + voltage);
        climberMotor.setControl(voltageRequest.withOutput(voltage));
        Logger.recordOutput("Climber/SetVoltage", voltage);
    }

    /** Stop climbing */
    public void stopClimb() {
        climberMotor.setControl(dutyCycleRequest.withOutput(0.0));
    }

    /** Move climber to start climb position */
    public void goToStartClimb() {
        double pos = Constants.CLIMBER_START_POSITION.get();
        System.out.println("ClimberSubsystem: goToStartClimb() called, position: " + pos);
        // For example, use position control if available, else just set voltage for now
        climberMotor.setControl(voltageRequest.withOutput(Constants.CLIMBER_VOLTAGE.get()));
        atStartPosition = true;
        climbing = false;
        Logger.recordOutput("Climber/GoToStartClimb", pos);
    }

    /** Move climber to zero position */
    public void goToZero() {
        double pos = Constants.CLIMBER_ZERO_POSITION.get();
        System.out.println("ClimberSubsystem: goToZero() called, position: " + pos);
        // For example, use position control if available, else just set voltage for now
        climberMotor.setControl(voltageRequest.withOutput(-Constants.CLIMBER_VOLTAGE.get()));
        atStartPosition = false;
        climbing = false;
        Logger.recordOutput("Climber/GoToZero", pos);
    }

    /** Start climbing for 5 seconds */
    public void startClimbFor5Sec() {
        System.out.println("ClimberSubsystem: startClimbFor5Sec() called");
        climbTimer.reset();
        climbTimer.start();
        climbing = true;
        atStartPosition = false;
    }

    /** Call this periodically to handle timed climb */
    public void handleTimedClimb() {
        if (climbing) {
            if (climbTimer.get() < 5.0) {
                climberMotor.setControl(voltageRequest.withOutput(Constants.CLIMBER_VOLTAGE.get()));
            } else {
                stopClimb();
                climbing = false;
                climbTimer.stop();
            }
        }
    }

    public boolean isAtStartPosition() {
        return atStartPosition;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Climber/Position", climberMotor.getPosition().getValue());
        Logger.recordOutput("Climber/Velocity", climberMotor.getVelocity().getValue());
        Logger.recordOutput("Climber/Current", climberMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Climber/Voltage", climberMotor.getSupplyVoltage().getValue());
        handleTimedClimb();
    }
}