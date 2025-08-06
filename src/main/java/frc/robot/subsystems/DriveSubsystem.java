// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.RobotConstants.TankConstants.*;

public class DriveSubsystem extends SubsystemBase {
    /**
     * Creates motors
     */
    TalonFX motorRight = new TalonFX(1, "rio");
    TalonFX motorLeft = new TalonFX(0, "rio");

    TalonFXConfigurator motorRightConfigurator = motorRight.getConfigurator();
    TalonFXConfigurator motorLeftConfigurator = motorLeft.getConfigurator();

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        motorLeftConfigurator.apply(motorOutputConfigs);
        motorRightConfigurator.apply(motorOutputConfigs);
    }

    /**
     * Run motor according to joystick input values.
     */
    public void setSpeeds(double leftSpeeds, double rightSpeeds) {
        motorLeft.set(leftSpeeds);
        motorRight.set(-rightSpeeds);
    }

    public void setRPS(double leftRPS, double rightRPS) {
        SmartDashboard.putNumber("DriveSubsystem/leftRPS",leftRPS);
        SmartDashboard.putNumber("DriveSubsystem/rightRPS",rightRPS);

        motorLeft.setControl(new VelocityVoltage(leftRPS));
        motorRight.setControl(new VelocityVoltage(-rightRPS));
    }

    /**
     *
     * @param forwardSpeed m/s
     * @param turningSpeed rad/s
     */

    // Run arcade drive based on setSpeeds
    public void setArcadeSpeed(double forwardSpeed, double turningSpeed) {
        double forwardRPS = forwardSpeed/2/Math.PI/WHEEL_RADIUS*GEAR_RATIO;
        double turningRPS = turningSpeed*WHEEL_TRACK/2*GEAR_RATIO;

        SmartDashboard.putNumber("DriveSubsystem/forwardSpeed",forwardSpeed);
        SmartDashboard.putNumber("DriveSubsystem/turningSpeed",turningSpeed);

        setRPS(forwardRPS+turningRPS,forwardRPS-turningRPS);
    }

    @Override
    public void periodic() {
//         This method will be called once per scheduler run
         motorLeft.getConfigurator().apply(new Slot0Configs().
                 withKP(TANK_PID.kP.get()).
                 withKI(TANK_PID.kI.get()).
                 withKD(TANK_PID.kD.get()));
         motorRight.getConfigurator().apply(new Slot0Configs().
                 withKP(TANK_PID.kP.get()).
                 withKI(TANK_PID.kI.get()).
                 withKD(TANK_PID.kD.get()));
    }
}
