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
import com.ctre.phoenix6.hardware.Pigeon2;

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

    // Gyro
    private static final Pigeon2 gyro = new Pigeon2(consts.CANID.GYRO);

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
    private double targetHeading = 0.0; // Target heading for turning operations
    private boolean isTurningInPlace = false; // Flag to track if we're currently turning in place
    private double leftStartPosition = 0.0; // Starting position of left motor when turning begins
    private double rightStartPosition = 0.0; // Starting position of right motor when turning begins

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
        // Calculate the distance each wheel needs to travel for the desired rotation
        double distance = (angle / 360.0) * Math.PI * consts.Superstructures.Chassis.TRACK_WIDTH; // in meters
        double rotations = distance * ROTATIONS_PER_METER;
        
        // Store starting positions
        leftStartPosition = leftMotor.getPosition().getValueAsDouble();
        rightStartPosition = rightMotor.getPosition().getValueAsDouble();
        
        // Set target positions for differential turning
        double leftTargetPosition = leftStartPosition + rotations;
        double rightTargetPosition = rightStartPosition - rotations;
        
        // Set the target heading for logging
        targetHeading = currentHeading + angle;
        isTurningInPlace = true;
        
        // Normalize target heading to 0-360 degrees
        while (targetHeading >= 360) {
            targetHeading -= 360;
        }
        while (targetHeading < 0) {
            targetHeading += 360;
        }
        
        // Set motor positions
        setLeftMotorPosition(leftTargetPosition);
        setRightMotorPosition(rightTargetPosition);
        
        Logger.recordOutput("DriveSubsystem/TargetHeading", targetHeading);
        Logger.recordOutput("DriveSubsystem/IsTurningInPlace", isTurningInPlace);
        Logger.recordOutput("DriveSubsystem/TurnAngle", angle);
        Logger.recordOutput("DriveSubsystem/TurnRotations", rotations);
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
        targetHeading = 0.0;
        isTurningInPlace = false;
    }
    
    /**
     * Handles PID control for turning in place
     * This method should be called in the periodic() method
     */
    private void handleTurningInPlace() {
        if (!isTurningInPlace) {
            return;
        }
        
        // Get current heading from gyro
        currentHeading = gyro.getYaw().getValueAsDouble();
        
        // Calculate the shortest angle difference
        double angleDifference = targetHeading - currentHeading;
        
        // Normalize to -180 to 180 degrees for shortest path
        if (angleDifference > 180) {
            angleDifference -= 360;
        } else if (angleDifference < -180) {
            angleDifference += 360;
        }
        
        // Check if we've reached the target within tolerance
        if (Math.abs(angleDifference) <= consts.PID.turnPID.tolerance.get()) {
            stopMotors();
            isTurningInPlace = false;
            currentHeading = targetHeading;
            Logger.recordOutput("DriveSubsystem/TurnComplete", true);
            return;
        }
        
        // Calculate PID output
        double kP = consts.PID.turnPID.kP.get();
        double kI = consts.PID.turnPID.kI.get();
        double kD = consts.PID.turnPID.kD.get();
        
        // Simple PID calculation (you might want to add integral and derivative terms)
        double output = angleDifference * kP;
        
        // Clamp output to reasonable limits
        output = Math.max(-consts.Limits.MAX_OUTPUT, Math.min(consts.Limits.MAX_OUTPUT, output));
        
        // Apply differential drive (left motor forward, right motor backward for turning)
        double leftSpeed = output;
        double rightSpeed = -output;
        
        // Set motor speeds
        leftMotor.setControl(velocityRequest.withVelocity(leftSpeed * 1000)); // Convert to RPM
        rightMotor.setControl(velocityRequest.withVelocity(rightSpeed * 1000));
        
        Logger.recordOutput("DriveSubsystem/AngleDifference", angleDifference);
        Logger.recordOutput("DriveSubsystem/TurnOutput", output);
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

        // Gyro
        Logger.recordOutput("Drive/Gyro/Yaw", gyro.getYaw().getValue());
        Logger.recordOutput("Drive/Gyro/Pitch", gyro.getPitch().getValue());
        Logger.recordOutput("Drive/Gyro/Roll", gyro.getRoll().getValue());
        Logger.recordOutput("Drive/CurrentHeading", currentHeading);
        Logger.recordOutput("Drive/TargetHeading", targetHeading);
        Logger.recordOutput("Drive/IsTurningInPlace", isTurningInPlace);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        checkAndUpdatePID();
        handleTurningInPlace(); // Handle PID-controlled turning
        log();
    }

    private void checkAndUpdatePID() {
        // Check if any PID values have changed
        boolean pidChanged = consts.PID.driveMotorPID.kP.hasChanged() ||
                           consts.PID.driveMotorPID.kI.hasChanged() ||
                           consts.PID.driveMotorPID.kD.hasChanged() ||
                           consts.PID.driveMotorPID.kS.hasChanged() ||
                           consts.PID.driveMotorPID.kV.hasChanged() ||
                           consts.PID.driveMotorPID.kA.hasChanged() ||
                           consts.PID.driveMotorPID.kG.hasChanged();

        // Check if Motion Magic values have changed
        boolean mmChanged = consts.MotionMagic.leftMotorMM.CRUISE_VELOCITY.hasChanged() ||
                          consts.MotionMagic.leftMotorMM.ACCELERATION.hasChanged() ||
                          consts.MotionMagic.leftMotorMM.JERK.hasChanged() ||
                          consts.MotionMagic.rightMotorMM.CRUISE_VELOCITY.hasChanged() ||
                          consts.MotionMagic.rightMotorMM.ACCELERATION.hasChanged() ||
                          consts.MotionMagic.rightMotorMM.JERK.hasChanged();

        // If any values changed, reconfigure the motors
        if (pidChanged || mmChanged) {
            Logger.recordOutput("Drive/PID/Reconfiguring", true);
            configureMotors();
            Logger.recordOutput("Drive/PID/kP", consts.PID.driveMotorPID.kP.get());
            Logger.recordOutput("Drive/PID/kI", consts.PID.driveMotorPID.kI.get());
            Logger.recordOutput("Drive/PID/kD", consts.PID.driveMotorPID.kD.get());
        } else {
            Logger.recordOutput("Drive/PID/Reconfiguring", false);
        }
    }
}
