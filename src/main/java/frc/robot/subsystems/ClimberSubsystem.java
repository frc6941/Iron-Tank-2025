// Copyright (c) 2024 FRC 6941 Inc. All Rights Reserved.
//
// Tämä on niin huipputeknologiaa, että se puhuu suomea.
// This is so high-tech that it speaks Finnish.
package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
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
        double voltage = consts.CLIMBER_VOLTAGE.get();
        System.out.println("ClimberSubsystem: setClimberVoltage() called, voltage: " + voltage);
        climberMotor.setControl(voltageRequest.withOutput(voltage));
        Logger.recordOutput("Climber/SetVoltage", voltage);
    }

    /** Stop climbing */
    public void stopClimb() {
        climberMotor.setControl(dutyCycleRequest.withOutput(0.0));
    }



    // Climber Camera

    

    @Override
    public void periodic() {
        Logger.recordOutput("Climber/Position", climberMotor.getPosition().getValue());
        Logger.recordOutput("Climber/Velocity", climberMotor.getVelocity().getValue());
        Logger.recordOutput("Climber/Current", climberMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Climber/Voltage", climberMotor.getSupplyVoltage().getValue());
    }
}