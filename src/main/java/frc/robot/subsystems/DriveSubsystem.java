package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.consts;
import frc.robot.consts.Drive.CheesyDrive;

public class DriveSubsystem extends SubsystemBase {

    // Motors
    private static final TalonFX motorLeft = new TalonFX(consts.CANID.MOTOR_LEFT);
    private static final TalonFX motorLeftFollower = new TalonFX(consts.CANID.MOTOR_LEFT_FOLLEWER);
    private static final TalonFX motorRight = new TalonFX(consts.CANID.MOTOR_RIGHT);
    private static final TalonFX motorRightFollower = new TalonFX(consts.CANID.MOTOR_RIGHT_FOLLOWER);

    // Gyro
    private static final WPI_PigeonIMU gyro = new WPI_PigeonIMU(consts.CANID.GYRO);

    // Controls
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final PIDController positionPID = new PIDController(consts.PID.positionPID.kP.get(), consts.PID.positionPID.kI.get(), consts.PID.positionPID.kD.get());

    // Motor Controllers for DifferentialDrive
    private static final MotorController moterLeftController = new TalonFXMotorController(motorLeft);
    private static final MotorController moterRightController = new TalonFXMotorController(motorRight);
    
    // Differential Drive
    private static final DifferentialDrive differentialDrive = new DifferentialDrive(moterLeftController, moterRightController);

    // *** NEW: State variables for Cheesy Drive logic ***
    private double mOldWheel = 0.0;
    private double mNegInertiaAccumulator = 0.0;

    public DriveSubsystem() {
        // Configure the motors
        configureMotors();

         // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig autoConfig = null;
        try{
        autoConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
            autoConfig, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );

    }

    // Custom motor controller class to wrap TalonFX for DifferentialDrive
    private static class TalonFXMotorController implements MotorController {
        private final TalonFX moter;
        
        public TalonFXMotorController(TalonFX moter) {
            this.moter = moter;
        }
        
        @Override
        public void set(double speed) {
            moter.setControl(new DutyCycleOut(speed));
        }
        
        @Override
        public double get() {
            return moter.getDutyCycle().getValueAsDouble();
        }
        
        @Override
        public void setInverted(boolean isInverted) {
            // Inversion is handled in moter configuration
        }
        
        @Override
        public boolean getInverted() {
            return false; // Inversion handled in moter config
        }
        
        @Override
        public void disable() {
            moter.setControl(new DutyCycleOut(0.0));
        }
        
        @Override
        public void stopMotor() {
            moter.setControl(new DutyCycleOut(0.0));
        }
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
        TalonFXConfiguration leftTalonFXConfig = generateTalonFXConfig(true, NeutralModeValue.Brake, slot0Configs);
        TalonFXConfiguration rightTalonFXConfig = generateTalonFXConfig(false, NeutralModeValue.Brake, slot0Configs);
        motorLeft.getConfigurator().apply(leftTalonFXConfig);
        motorRight.getConfigurator().apply(rightTalonFXConfig);
        motorLeftFollower.getConfigurator().apply(leftTalonFXConfig);
        motorRightFollower.getConfigurator().apply(rightTalonFXConfig);
        motorLeftFollower.setControl(new Follower(motorLeft.getDeviceID(), false));
        motorRightFollower.setControl(new Follower(motorRight.getDeviceID(), false));
    }

    public TalonFXConfiguration generateTalonFXConfig (
        boolean inverted, 
        NeutralModeValue defaultNeutralMode, 
        Slot0Configs slot0
        ) {
        TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
        talonFXConfig.MotorOutput.Inverted = inverted? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        talonFXConfig.MotorOutput.NeutralMode = defaultNeutralMode;
        talonFXConfig.Slot0 = slot0;
        // Apply current limits from constants
        talonFXConfig.CurrentLimits.SupplyCurrentLimit = consts.Limits.Chassis.DRIVE_SUPPLY_CURRENT_LIMIT;
        talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = consts.Limits.Chassis.DRIVE_STATOR_CURRENT_LIMIT;
        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
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
    
    // Remove the deprecated arcadeDrive method as it is not used.

    // *** NEW: Cheesy Drive implementation with negative inertia and quick-turn ***
    public void cheesyDrive(double forward, double rotation, boolean isQuickTurn) {
        // Apply deadband from constants
        double throttle = Math.abs(forward) < consts.Drive.DEADBAND ? 0.0 : forward;
        double wheel = Math.abs(rotation) < consts.Drive.DEADBAND ? 0.0 : rotation;

        // --- Negative Inertia Logic ---
        double negInertia = wheel - mOldWheel;
        mOldWheel = wheel;

        double negInertiaScalar;
        if (wheel * negInertia > 0) {
            negInertiaScalar = consts.Drive.CheesyDrive.NEG_INERTIA_SCALAR;
        } else {
            if (Math.abs(wheel) > consts.Drive.CheesyDrive.NEG_INERTIA_THRESHOLD) {
                negInertiaScalar = consts.Drive.CheesyDrive.NEG_INERTIA_TURN_SCALAR;
            } else {
                negInertiaScalar = consts.Drive.CheesyDrive.NEG_INERTia_CLOSE_SCALAR;
            }
        }
        
        double negInertiaPower = negInertia * negInertiaScalar;
        mNegInertiaAccumulator += negInertiaPower;

        // Apply the corrective power to the steering command
        wheel += mNegInertiaAccumulator;

        // Decay the accumulator
        if (mNegInertiaAccumulator > 1) {
            mNegInertiaAccumulator -= 1;
        } else if (mNegInertiaAccumulator < -1) {
            mNegInertiaAccumulator += 1;
        } else {
            mNegInertiaAccumulator = 0;
        }

        double leftPwm, rightPwm;

        if (isQuickTurn) {
            // SWAPPED: Fix left/right reversal
            leftPwm = -wheel;
            rightPwm = wheel;
        } else {
            double overPower;
            double angularPower = Math.abs(throttle) * wheel;
            // SWAPPED: Fix left/right reversal
            leftPwm = throttle - angularPower;
            rightPwm = throttle + angularPower;

            if (leftPwm > 1.0) {
                overPower = leftPwm - 1.0;
                rightPwm -= overPower;
                leftPwm = 1.0;
            } else if (rightPwm > 1.0) {
                overPower = rightPwm - 1.0;
                leftPwm -= overPower;
                rightPwm = 1.0;
            } else if (leftPwm < -1.0) {
                overPower = leftPwm + 1.0;
                rightPwm -= overPower;
                leftPwm = -1.0;
            } else if (rightPwm < -1.0) {
                overPower = rightPwm + 1.0;
                leftPwm -= overPower;
                rightPwm = -1.0;
            }
        }



        // Use tankDrive to set the final motor outputs.
        tankDrive(leftPwm, rightPwm);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        // Apply speed limiting
        leftSpeed = Math.max(-consts.Limits.Chassis.MAX_OUTPUT, Math.min(consts.Limits.Chassis.MAX_OUTPUT, leftSpeed));
        rightSpeed = Math.max(-consts.Limits.Chassis.MAX_OUTPUT, Math.min(consts.Limits.Chassis.MAX_OUTPUT, rightSpeed));
        
        // Use DifferentialDrive, ensuring inputs are not squared as calculations are already done
        differentialDrive.tankDrive(leftSpeed, rightSpeed, false);
    }
    

    public static double getDistance(TalonFX motor) {
        return motor.getPosition().getValueAsDouble()*consts.Superstructures.Chassis.METERS_PER_ROTATION;
    }

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(consts.Superstructures.Chassis.TRACK_WIDTH);

    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
        gyro.getRotation2d(),
        getDistance(motorLeft),
        getDistance(motorRight));

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void updateOdometry(){
        var gyroAngle = gyro.getRotation2d();
        odometry.update(gyroAngle, getDistance(motorLeft),getDistance(motorRight));
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getDistance(motorLeft),getDistance(motorRight), pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(
            Units.rotationsToRadians(motorLeft.getVelocity().getValueAsDouble()), 
            Units.rotationsToRadians(motorRight.getVelocity().getValueAsDouble()));
        return kinematics.toChassisSpeeds(speeds);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        setLeftMotorVelocity(wheelSpeeds.leftMetersPerSecond);
        setRightMotorVelocity(wheelSpeeds.rightMetersPerSecond);
    }

    public void setLeftMotorVelocity(double velocity) {
        double targetRPS = velocity * consts.Superstructures.Chassis.ROTATIONS_PER_METER;
        setLeftMotorRPM(targetRPS);
    }

    public void setRightMotorVelocity(double velocity) {
        double targetRPS = velocity * consts.Superstructures.Chassis.ROTATIONS_PER_METER;
        setRightMotorRPM(targetRPS);
    }

    public void setMotorVelocity(double velocity) {
        setLeftMotorVelocity(velocity);
        setRightMotorVelocity(velocity);
    }

    public void setLeftMotorPosition(double position) {
        // Convert position to velocity for simple control
        double currentPosition = motorLeft.getPosition().getValueAsDouble();
        double velocity = positionPID.calculate(currentPosition, position);
        motorLeft.setControl(velocityRequest.withVelocity(velocity));
    }

    public void setRightMotorPosition(double position) {
        // Convert position to velocity for simple control
        double currentPosition = motorRight.getPosition().getValueAsDouble();
        double velocity = positionPID.calculate(currentPosition, position);
        motorRight.setControl(velocityRequest.withVelocity(velocity));
    }

    public void setMotorPosition(double position) {
        setLeftMotorPosition(position);
        setRightMotorPosition(position);
    }

    public void turnInPlace(double angle) {
        // Calculate the distance each wheel needs to travel for the desired rotation
        double distance = (angle / 360.0) * Math.PI * consts.Superstructures.Chassis.TRACK_WIDTH; // in meters
        double rotations = distance * consts.Superstructures.Chassis.ROTATIONS_PER_METER;
        
        // Store starting positions
        double leftStartPosition = motorLeft.getPosition().getValueAsDouble();
        double rightStartPosition = motorRight.getPosition().getValueAsDouble();
        
        // Set target positions for differential turning
        double leftTargetPosition = leftStartPosition + rotations;
        double rightTargetPosition = rightStartPosition - rotations;
        
        // Set motor positions
        setLeftMotorPosition(leftTargetPosition);
        setRightMotorPosition(rightTargetPosition);
        
        Logger.recordOutput("DriveSubsystem/TurnAngle", angle);
        Logger.recordOutput("DriveSubsystem/TurnRotations", rotations);
    }

    public void driveLeftDistance(double distance) {
        double deltaRotations = distance * consts.Superstructures.Chassis.ROTATIONS_PER_METER;
        double targetPosition = motorLeft.getPosition().getValueAsDouble() + deltaRotations;
        setLeftMotorPosition(targetPosition);
    }

    public void driveRightDistance(double distance) {
        double deltaRotations = distance * consts.Superstructures.Chassis.ROTATIONS_PER_METER;
        double targetPosition = motorRight.getPosition().getValueAsDouble() + deltaRotations;
        setRightMotorPosition(targetPosition);
    }

    public void driveDistance(double distance) {
        driveLeftDistance(distance);
        driveRightDistance(distance);
    }

    public void setLeftMotorRPM(double rps) {
        motorLeft.setControl(velocityRequest.withVelocity(rps));
    }

    public void setRightMotorRPM(double rps) {
        motorRight.setControl(velocityRequest.withVelocity(rps));
    }

    public void setMotorPRM(double rps) {
        setLeftMotorRPM(rps);
        setRightMotorRPM(rps);
    }

    public void stopLeftMotor() {
        motorLeft.setControl(dutyCycleRequest.withOutput(0.0));
    }

    public void stopRightMotor() {
        motorRight.setControl(dutyCycleRequest.withOutput(0.0));
    }

    public void stopMotors() {
        stopLeftMotor();
        stopRightMotor();
    }

    private void updatePID() {
        // Check if any PID values have changed
        boolean pidChanged = consts.PID.driveMotorPID.kP.hasChanged() ||
                           consts.PID.driveMotorPID.kI.hasChanged() ||
                           consts.PID.driveMotorPID.kD.hasChanged() ||
                           consts.PID.driveMotorPID.kS.hasChanged() ||
                           consts.PID.driveMotorPID.kV.hasChanged() ||
                           consts.PID.driveMotorPID.kA.hasChanged() ||
                           consts.PID.driveMotorPID.kG.hasChanged();

        // If any values changed, reconfigure the motors
        if (pidChanged) {
            Logger.recordOutput("Drive/PID/Reconfiguring", true);
            configureMotors();
            Logger.recordOutput("Drive/PID/kP", consts.PID.driveMotorPID.kP.get());
            Logger.recordOutput("Drive/PID/kI", consts.PID.driveMotorPID.kI.get());
            Logger.recordOutput("Drive/PID/kD", consts.PID.driveMotorPID.kD.get());
        } else {
            Logger.recordOutput("Drive/PID/Reconfiguring", false);
        }
    }
    
    public void log() {
        // Left Motors
        // Master
        Logger.recordOutput("Drive/Left/Master/Position", motorLeft.getPosition().getValue());
        Logger.recordOutput("Drive/Left/Master/Velocity", motorLeft.getVelocity().getValue());
        Logger.recordOutput("Drive/Left/Master/Acceleration", motorLeft.getAcceleration().getValue());
        Logger.recordOutput("Drive/Left/Master/Current", motorLeft.getStatorCurrent().getValue());
        Logger.recordOutput("Drive/Left/Master/Voltage", motorLeft.getSupplyVoltage().getValue());
        
        // Follower
        Logger.recordOutput("Drive/Left/Follower/Position", motorLeftFollower.getPosition().getValue());
        Logger.recordOutput("Drive/Left/Follower/Velocity", motorLeftFollower.getVelocity().getValue());
        Logger.recordOutput("Drive/Left/Follower/Acceleration", motorLeftFollower.getAcceleration().getValue());
        Logger.recordOutput("Drive/Left/Follower/Current", motorLeftFollower.getStatorCurrent().getValue());
        Logger.recordOutput("Drive/Left/Follower/Voltage", motorLeftFollower.getSupplyVoltage().getValue());

        // Right Motors
        // Master
        Logger.recordOutput("Drive/Right/Master/Position", motorRight.getPosition().getValue());
        Logger.recordOutput("Drive/Right/Master/Velocity", motorRight.getVelocity().getValue());
        Logger.recordOutput("Drive/Right/Master/Acceleration", motorRight.getAcceleration().getValue());
        Logger.recordOutput("Drive/Right/Master/Current", motorRight.getStatorCurrent().getValue());
        Logger.recordOutput("Drive/Right/Master/Voltage", motorRight.getSupplyVoltage().getValue());

        // // Follower
        Logger.recordOutput("Drive/Right/Follower/Position", motorRightFollower.getPosition().getValue());
        Logger.recordOutput("Drive/Right/Follower/Velocity", motorRightFollower.getVelocity().getValue());
        Logger.recordOutput("Drive/Right/Follower/Acceleration", motorRightFollower.getAcceleration().getValue());
        Logger.recordOutput("Drive/Right/Follower/Current", motorRightFollower.getStatorCurrent().getValue());
        Logger.recordOutput("Drive/Right/Follower/Voltage", motorRightFollower.getSupplyVoltage().getValue());

        // Wheel-specific logging (combined master + follower data)
        // Left wheel velocity (average of master and follower)
        double leftWheelVelocity = (motorLeft.getVelocity().getValueAsDouble() + motorLeftFollower.getVelocity().getValueAsDouble()) / 2.0;
        Logger.recordOutput("Drive/Wheels/Left/Velocity", leftWheelVelocity);
        
        // Right wheel velocity (average of master and follower)
        double rightWheelVelocity = (motorRight.getVelocity().getValueAsDouble() + motorRightFollower.getVelocity().getValueAsDouble()) / 2.0;
        Logger.recordOutput("Drive/Wheels/Right/Velocity", rightWheelVelocity);
        
        // Left wheel current (sum of master and follower)
        double leftWheelCurrent = motorLeft.getStatorCurrent().getValueAsDouble() + motorLeftFollower.getStatorCurrent().getValueAsDouble();
        Logger.recordOutput("Drive/Wheels/Left/Current", leftWheelCurrent);
        
        // Right wheel current (sum of master and follower)
        double rightWheelCurrent = motorRight.getStatorCurrent().getValueAsDouble() + motorRightFollower.getStatorCurrent().getValueAsDouble();
        Logger.recordOutput("Drive/Wheels/Right/Current", rightWheelCurrent);
        
        // Wheel velocity in meters per second (converted from rotations per second)
        double leftWheelVelocityMPS = leftWheelVelocity * consts.Superstructures.Chassis.METERS_PER_ROTATION;
        double rightWheelVelocityMPS = rightWheelVelocity * consts.Superstructures.Chassis.METERS_PER_ROTATION;
        Logger.recordOutput("Drive/Wheels/Left/VelocityMPS", leftWheelVelocityMPS);
        Logger.recordOutput("Drive/Wheels/Right/VelocityMPS", rightWheelVelocityMPS);
        
        // Average wheel velocity for overall robot speed
        double averageWheelVelocityMPS = (leftWheelVelocityMPS + rightWheelVelocityMPS) / 2.0;
        Logger.recordOutput("Drive/Wheels/AverageVelocityMPS", averageWheelVelocityMPS);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        updatePID();
        log();
        updateOdometry();
    }
}