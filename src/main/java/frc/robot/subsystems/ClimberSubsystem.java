// Copyright (c) 2024 FRC 6941 Inc. All Rights Reserved.
//
// Tämä on niin huipputeknologiaa, että se puhuu suomea.
// This is so high-tech that it speaks Finnish.
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
import frc.robot.consts;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climberMotor = new TalonFX(consts.CANID.CLIMBER_MOTOR);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    // // State variables for position control
    // private boolean movingToPosition = false;
    // private double targetPosition = 0.0;
    // private boolean movingToStop = false;

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
    // public void setClimberVoltage() {
    //     double voltage = consts.CLIMBER_VOLTAGE.get();
    //     System.out.println("ClimberSubsystem: setClimberVoltage() called, voltage: " + voltage);
    //     climberMotor.setControl(voltageRequest.withOutput(voltage));
    //     Logger.recordOutput("Climber/SetVoltage", voltage);
    // }

    /** Stop climbing */
    // public void stopClimb() {
    //     climberMotor.setControl(dutyCycleRequest.withOutput(0.0));
    // }

    // /** Move climber to zero position using voltage control */
    // public void goToZeroPosition() {
    //     double target = frc.robot.consts.CLIMBER_ZERO_POSITION.get();
    //     System.out.println("ClimberSubsystem: goToZeroPosition() called, target: " + target);
    //     targetPosition = target;
    //     movingToPosition = true;
    //     movingToStop = false;
    // }

    // /** Move climber to start position using voltage control */
    // public void goToStartPosition() {
    //     double target = frc.robot.consts.CLIMBER_START_POSITION.get();
    //     System.out.println("ClimberSubsystem: goToStartPosition() called, target: " + target);
    //     targetPosition = target;
    //     movingToPosition = true;
    //     movingToStop = false;
    // }

    // /** Move climber to stop position using voltage control */
    // public void goToStopPosition() {
    //     double target = frc.robot.consts.CLIMBER_STOP_POSITION.get();
    //     System.out.println("ClimberSubsystem: goToStopPosition() called, target: " + target);
    //     targetPosition = target;
    //     movingToPosition = true;
    //     movingToStop = false;
    // }
    public void moveUp() {
        climberMotor.setControl(voltageRequest.withOutput(4.0));
    }
    public void moveDown() {
        climberMotor.setControl(voltageRequest.withOutput(-4.0));
    }
    public void stop() {
        climberMotor.setControl(dutyCycleRequest.withOutput(0.0));
    }
    /** Handle position-based movement in periodic */
    // private void handlePositionMovement() {
    //     if (movingToPosition) {
    //         double currentPosition = climberMotor.getPosition().getValueAsDouble();
    //         double positionError = targetPosition - currentPosition;
    //         double tolerance = 1.0; // Position tolerance in rotations
            
    //         if (Math.abs(positionError) > tolerance) {                // Apply voltage based on target position only
    //             if (targetPosition == frc.robot.consts.CLIMBER_START_POSITION.get()) {
    //                 // Moving to start position - always use positive voltage
    //                 climberMotor.setControl(voltageRequest.withOutput(4.0));
    //             } else {
    //                 // Moving to zero or stop position - always use negative voltage
    //                 climberMotor.setControl(voltageRequest.withOutput(-4.0));
    //             }
    //         } else {                // Reached target position
    //             System.out.println("ClimberSubsystem: Reached target position: " + targetPosition);
    //             climberMotor.setControl(dutyCycleRequest.withOutput(0.0));
    //             movingToPosition = false;
    //         }
    //     }
    // }

    @Override
    public void periodic() {
        Logger.recordOutput("Climber/Position", climberMotor.getPosition().getValue());
        Logger.recordOutput("Climber/Velocity", climberMotor.getVelocity().getValue());
        Logger.recordOutput("Climber/Current", climberMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Climber/Voltage", climberMotor.getSupplyVoltage().getValue());
        // handlePositionMovement();
    }
}