package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.consts;

public class DriveSubsystem extends SubsystemBase {

    /* Definitions */
    // T1 constants that are either calculated or retrieved from the robot's hardware.
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * consts.Superstructures.Chassis.WHEEL_DIAMETER; // in meters
    private static final double METERS_PER_ROTATION = WHEEL_CIRCUMFERENCE / consts.Superstructures.Chassis.GEAR_RATIO;
    private static final double ROTATIONS_PER_METER = 1.0 / METERS_PER_ROTATION;

    // Motors
    private static final TalonFX leftMotor = new TalonFX(consts.CANID.LEFTMOTOR);
    private static final TalonFX leftMotorFollower = new TalonFX(consts.CANID.LEFTMOTORFOLLEWER);
    private static final TalonFX rightMotor = new TalonFX(consts.CANID.RIGHTMOTOR);
    private static final TalonFX rightMotorFollower = new TalonFX(consts.CANID.RIGHTMOTORFOLLOWER);

    // Controls
    private static final MotorController leftMotorController = new PWMTalonFX(leftMotor.getDeviceID());
    private static final MotorController rightMotorController = new PWMTalonFX(rightMotor.getDeviceID());
    private static final MotionMagicVoltage leftMMRequest = new MotionMagicVoltage(0);
    private static final MotionMagicVoltage rightMMRequest = new MotionMagicVoltage(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    // Differential Drive
    private static final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorController, rightMotorController);

    // Attitude Variables
    private double currentHeading = 0.0; // Current heading of the robot in degrees. Initializes at 0 degrees.

    public DriveSubsystem() {
        // Configure the motors
        configureMotors();
    }

    public void configureMotors() {
        // Initialize the drive subsystem here
        Slot0Configs slot0Configs = generateSlot0Configs(
            consts.PID.driveMotorPID.kP.get(),
            consts.PID.driveMotorPID.kI.get(),
            consts.PID.driveMotorPID.kD.get(),
            consts.PID.driveMotorPID.kS.get(),
            consts.PID.driveMotorPID.kV.get(),
            consts.PID.driveMotorPID.kA.get(),
            consts.PID.driveMotorPID.kG.get()
        );
        MotionMagicConfigs leftMMConfigs = generateMotionMagicConfigs(
            consts.MotionMagic.leftMotorMM.CRUISE_VELOCITY.get(),
            consts.MotionMagic.leftMotorMM.ACCELERATION.get(),
            consts.MotionMagic.leftMotorMM.JERK.get()
        );
        MotionMagicConfigs rightMMConfigs = generateMotionMagicConfigs(
            consts.MotionMagic.rightMotorMM.CRUISE_VELOCITY.get(),
            consts.MotionMagic.rightMotorMM.ACCELERATION.get(),
            consts.MotionMagic.rightMotorMM.JERK.get()
        );
        TalonFXConfiguration leftTalonFXConfig = generateTalonFXConfig(false, NeutralModeValue.Brake, slot0Configs, leftMMConfigs);
        TalonFXConfiguration rightTalonFXConfig = generateTalonFXConfig(true, NeutralModeValue.Brake, slot0Configs, rightMMConfigs);
        leftMotor.getConfigurator().apply(leftTalonFXConfig);
        rightMotor.getConfigurator().apply(rightTalonFXConfig);
        leftMotorFollower.setControl(new Follower(leftMotor.getDeviceID(), true));
        rightMotorFollower.setControl(new Follower(rightMotor.getDeviceID(), true));
    }

    public TalonFXConfiguration generateTalonFXConfig (
        boolean inverted, 
        NeutralModeValue defaultNeutralMode, 
        Slot0Configs slot0, 
        MotionMagicConfigs motionMagicConfig
        ) {
        TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
        talonFXConfig.MotorOutput.Inverted = inverted? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        talonFXConfig.MotorOutput.NeutralMode = defaultNeutralMode;

        talonFXConfig.Slot0 = slot0;
        talonFXConfig.MotionMagic = motionMagicConfig;

        return talonFXConfig;
    }

    public Slot0Configs generateSlot0Configs(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;
        slot0.kS = kS;
        slot0.kV = kV;
        slot0.kA = kA;
        slot0.kG = kG;
        return slot0;
    }

    public MotionMagicConfigs generateMotionMagicConfigs(
        double cruiseVelocity, 
        double acceleration, 
        double jerk
    ) {
        MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
        mmConfigs.MotionMagicCruiseVelocity = cruiseVelocity;
        mmConfigs.MotionMagicAcceleration = acceleration;
        mmConfigs.MotionMagicJerk = jerk;
        return mmConfigs;
    }

    public void arcadeDrive(double forward, double rotation) {
        differentialDrive.arcadeDrive(forward * consts.Limits.MAX_OUTPUT, rotation * consts.Limits.MAX_OUTPUT);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        differentialDrive.tankDrive(leftSpeed * consts.Limits.MAX_OUTPUT, rightSpeed * consts.Limits.MAX_OUTPUT);
    }

    public void setLeftMotorVelocity(double velocity) {
        double targetRPS = velocity * ROTATIONS_PER_METER;
        setLeftMotorRPM(targetRPS);
    }

    public void setRightMotorVelocity(double velocity) {
        double targetRPS = velocity * ROTATIONS_PER_METER;
        setRightMotorRPM(targetRPS);
    }

    public void setMotorVelocity(double velocity) {
        setLeftMotorVelocity(velocity);
        setRightMotorVelocity(velocity);
    }

    public void setLeftMotorPosition(double position) {
        leftMotor.setControl(leftMMRequest.withPosition(position));
    }

    public void setRightMotorPosition(double position) {
        rightMotor.setControl(rightMMRequest.withPosition(position));
    }

    public void setMotorPosition(double position) {
        setLeftMotorPosition(position);
        setRightMotorPosition(position);
    }

    public void turnInPlace(double angle) {
        // Calculate the distance each wheel needs to travel.
        double distance = (angle / 360.0) * Math.PI * consts.Superstructures.Chassis.TRACK_WIDTH; // in meters
        double rotations = distance * ROTATIONS_PER_METER;
        // Set the target position for each motor.
        double leftTargetPosition = leftMotor.getPosition().getValueAsDouble() + rotations;
        double rightTargetPosition = rightMotor.getPosition().getValueAsDouble() - rotations;
        setLeftMotorPosition(leftTargetPosition);
        setRightMotorPosition(rightTargetPosition);
    }

    public void turnToFace(double angle) {
        // Calculate the difference between the current heading and the target angle.
        double angleDifference = angle - currentHeading;
        // Normalize the angle difference to be within -180 to 180 degrees.
        if (angleDifference > 180) {
            angleDifference -= 360;
        } else if (angleDifference < -180) {
            angleDifference += 360;
        }
        // Turn in place to face the target angle.
        turnInPlace(angleDifference);
        // Update the current heading to the target angle.
        currentHeading = angle;
    }

    public void driveLeftDistance(double distance) {
        double deltaRotations = distance * ROTATIONS_PER_METER;
        double targetPosition = leftMotor.getPosition().getValueAsDouble() + deltaRotations;
        setLeftMotorPosition(targetPosition);
    }

    public void driveRightDistance(double distance) {
        double deltaRotations = distance * ROTATIONS_PER_METER;
        double targetPosition = rightMotor.getPosition().getValueAsDouble() + deltaRotations;
        setRightMotorPosition(targetPosition);
    }

    public void driveDistance(double distance) {
        driveLeftDistance(distance);
        driveRightDistance(distance);
    }

    public void setLeftMotorRPM(double rps) {
        leftMotor.setControl(velocityRequest.withVelocity(rps));
    }

    public void setRightMotorRPM(double rps) {
        rightMotor.setControl(velocityRequest.withVelocity(rps));
    }

    public void setMotorPRM(double rps) {
        setLeftMotorRPM(rps);
        setRightMotorRPM(rps);
    }

    public void stopLeftMotor() {
        leftMotor.setControl(dutyCycleRequest.withOutput(0.0));
    }

    public void stopRightMotor() {
        rightMotor.setControl(dutyCycleRequest.withOutput(0.0));
    }

    public void stopMotors() {
        stopLeftMotor();
        stopRightMotor();
    }

    public void resetFacingAngle() {
        currentHeading = 0.0;
    }

    public void log() {
        // Left Motors
        // Master
        Logger.recordOutput("Drive/Left/Master/Position", leftMotor.getPosition().getValue());
        Logger.recordOutput("Drive/Left/Master/Velocity", leftMotor.getVelocity().getValue());
        Logger.recordOutput("Drive/Left/Master/Acceleration", leftMotor.getAcceleration().getValue());
        Logger.recordOutput("Drive/Left/Master/Current", leftMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Drive/Left/Master/Voltage", leftMotor.getSupplyVoltage().getValue());
        
        // Follower
        Logger.recordOutput("Drive/Left/Follower/Position", leftMotorFollower.getPosition().getValue());
        Logger.recordOutput("Drive/Left/Follower/Velocity", leftMotorFollower.getVelocity().getValue());
        Logger.recordOutput("Drive/Left/Follower/Acceleration", leftMotorFollower.getAcceleration().getValue());
        Logger.recordOutput("Drive/Left/Follower/Current", leftMotorFollower.getStatorCurrent().getValue());
        Logger.recordOutput("Drive/Left/Follower/Voltage", leftMotorFollower.getSupplyVoltage().getValue());

        // Right Motors
        // Master
        Logger.recordOutput("Drive/Right/Master/Position", rightMotor.getPosition().getValue());
        Logger.recordOutput("Drive/Right/Master/Velocity", rightMotor.getVelocity().getValue());
        Logger.recordOutput("Drive/Right/Master/Acceleration", rightMotor.getAcceleration().getValue());
        Logger.recordOutput("Drive/Right/Master/Current", rightMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Drive/Right/Master/Voltage", rightMotor.getSupplyVoltage().getValue());

        // Follower
        Logger.recordOutput("Drive/Right/Follower/Position", rightMotorFollower.getPosition().getValue());
        Logger.recordOutput("Drive/Right/Follower/Velocity", rightMotorFollower.getVelocity().getValue());
        Logger.recordOutput("Drive/Right/Follower/Acceleration", rightMotorFollower.getAcceleration().getValue());
        Logger.recordOutput("Drive/Right/Follower/Current", rightMotorFollower.getStatorCurrent().getValue());
        Logger.recordOutput("Drive/Right/Follower/Voltage", rightMotorFollower.getSupplyVoltage().getValue());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        log();
    }
}
