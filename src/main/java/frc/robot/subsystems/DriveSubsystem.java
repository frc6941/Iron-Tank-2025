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
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.consts;
import frc.robot.utils.TunableNumber;

public class DriveSubsystem extends SubsystemBase {

    // Motors
    private static final TalonFX leftMotor = new TalonFX(consts.CANID.MOTOR_LEFT);
    private static final TalonFX leftMotorFollower = new TalonFX(consts.CANID.MOTOR_LEFT_FOLLEWER);
    private static final TalonFX rightMotor = new TalonFX(consts.CANID.MOTOR_RIGHT);
    private static final TalonFX rightMotorFollower = new TalonFX(consts.CANID.MOTOR_RIGHT_FOLLOWER);

    // Gyro
    private static final WPI_PigeonIMU gyro = new WPI_PigeonIMU(consts.CANID.GYRO);

    // Controls
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final PIDController positionPID = new PIDController(consts.PID.positionPID.kP.get(), consts.PID.positionPID.kI.get(), consts.PID.positionPID.kD.get());

    // Motor Controllers for DifferentialDrive
    private static final MotorController leftMotorController = new TalonFXMotorController(leftMotor);
    private static final MotorController rightMotorController = new TalonFXMotorController(rightMotor);
    
    // Differential Drive
    private static final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorController, rightMotorController);

    // *** NEW: State variables for Cheesy Drive logic ***
    private double mOldWheel = 0.0;
    private double mNegInertiaAccumulator = 0.0;

    public static final TunableNumber CLIMBER_VOLTAGE = new TunableNumber("climber_voltage", 4.0); // default value, adjust as needed

    public DriveSubsystem() {
        // Configure the motors
        configureMotors();

         // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try{
        config = RobotConfig.fromGUISettings();
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
            config, // The robot configuration
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

    public Pose2d getPose() {

    }

    // Custom motor controller class to wrap TalonFX for DifferentialDrive
    private static class TalonFXMotorController implements MotorController {
        private final TalonFX motor;
        
        public TalonFXMotorController(TalonFX motor) {
            this.motor = motor;
        }
        
        @Override
        public void set(double speed) {
            motor.setControl(new DutyCycleOut(speed));
        }
        
        @Override
        public double get() {
            return motor.getDutyCycle().getValueAsDouble();
        }
        
        @Override
        public void setInverted(boolean isInverted) {
            // Inversion is handled in motor configuration
        }
        
        @Override
        public boolean getInverted() {
            return false; // Inversion handled in motor config
        }
        
        @Override
        public void disable() {
            motor.setControl(new DutyCycleOut(0.0));
        }
        
        @Override
        public void stopMotor() {
            motor.setControl(new DutyCycleOut(0.0));
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
        leftMotor.getConfigurator().apply(leftTalonFXConfig);
        rightMotor.getConfigurator().apply(rightTalonFXConfig);
        leftMotorFollower.getConfigurator().apply(leftTalonFXConfig);
        rightMotorFollower.getConfigurator().apply(rightTalonFXConfig);
        leftMotorFollower.setControl(new Follower(leftMotor.getDeviceID(), false));
        rightMotorFollower.setControl(new Follower(rightMotor.getDeviceID(), false));
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
    
    /**
     * @deprecated Use cheesyDrive() instead for better operator control.
     */
    @Deprecated
    public void arcadeDrive(double forward, double rotation) {
        differentialDrive.arcadeDrive(forward, rotation);
    }

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

            DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), , null)
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
        double currentPosition = leftMotor.getPosition().getValueAsDouble();
        double velocity = positionPID.calculate(currentPosition, position);
        leftMotor.setControl(velocityRequest.withVelocity(velocity));
    }

    public void setRightMotorPosition(double position) {
        // Convert position to velocity for simple control
        double currentPosition = rightMotor.getPosition().getValueAsDouble();
        double velocity = positionPID.calculate(currentPosition, position);
        rightMotor.setControl(velocityRequest.withVelocity(velocity));
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
        double leftStartPosition = leftMotor.getPosition().getValueAsDouble();
        double rightStartPosition = rightMotor.getPosition().getValueAsDouble();
        
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
        double targetPosition = leftMotor.getPosition().getValueAsDouble() + deltaRotations;
        setLeftMotorPosition(targetPosition);
    }

    public void driveRightDistance(double distance) {
        double deltaRotations = distance * consts.Superstructures.Chassis.ROTATIONS_PER_METER;
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

        // // Follower
        Logger.recordOutput("Drive/Right/Follower/Position", rightMotorFollower.getPosition().getValue());
        Logger.recordOutput("Drive/Right/Follower/Velocity", rightMotorFollower.getVelocity().getValue());
        Logger.recordOutput("Drive/Right/Follower/Acceleration", rightMotorFollower.getAcceleration().getValue());
        Logger.recordOutput("Drive/Right/Follower/Current", rightMotorFollower.getStatorCurrent().getValue());
        Logger.recordOutput("Drive/Right/Follower/Voltage", rightMotorFollower.getSupplyVoltage().getValue());

        // Wheel-specific logging (combined master + follower data)
        // Left wheel velocity (average of master and follower)
        double leftWheelVelocity = (leftMotor.getVelocity().getValueAsDouble() + leftMotorFollower.getVelocity().getValueAsDouble()) / 2.0;
        Logger.recordOutput("Drive/Wheels/Left/Velocity", leftWheelVelocity);
        
        // Right wheel velocity (average of master and follower)
        double rightWheelVelocity = (rightMotor.getVelocity().getValueAsDouble() + rightMotorFollower.getVelocity().getValueAsDouble()) / 2.0;
        Logger.recordOutput("Drive/Wheels/Right/Velocity", rightWheelVelocity);
        
        // Left wheel current (sum of master and follower)
        double leftWheelCurrent = leftMotor.getStatorCurrent().getValueAsDouble() + leftMotorFollower.getStatorCurrent().getValueAsDouble();
        Logger.recordOutput("Drive/Wheels/Left/Current", leftWheelCurrent);
        
        // Right wheel current (sum of master and follower)
        double rightWheelCurrent = rightMotor.getStatorCurrent().getValueAsDouble() + rightMotorFollower.getStatorCurrent().getValueAsDouble();
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
    }
}