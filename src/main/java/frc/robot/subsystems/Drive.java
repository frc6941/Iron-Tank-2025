package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class Drive extends SubsystemBase {
  private static final String TAG = "Drive";

  private final TalonFX motorLeft = new TalonFX(Constants.CANID.MOTOR_LEFT);
  private final TalonFX motorLeftFollower = new TalonFX(Constants.CANID.MOTOR_LEFT_FOLLEWER);
  private final TalonFX motorRight = new TalonFX(Constants.CANID.MOTOR_RIGHT);
  private final TalonFX motorRightFollower = new TalonFX(Constants.CANID.MOTOR_RIGHT_FOLLOWER);

  private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(Constants.CANID.GYRO);

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final VelocityVoltage leftVelocityRequest = new VelocityVoltage(0);
  private final VelocityVoltage rightVelocityRequest = new VelocityVoltage(0);

  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDrivePoseEstimator poseEstimator;

  public Drive() {
    kinematics = new DifferentialDriveKinematics(Constants.Drive.TRACK_WIDTH);
    poseEstimator = new DifferentialDrivePoseEstimator(
        kinematics,
//        gyro.getRotation2d(),
        new Rotation2d(),
        motorAngleToDrivePosition(motorLeft.getPosition().getValue()).in(Meters),
        motorAngleToDrivePosition(motorRight.getPosition().getValue()).in(Meters),
        new Pose2d()
    );

    configure();
  }

  private static Distance motorAngleToDrivePosition(Angle angle) {
    return Constants.Drive.WHEEL_DIAMETER.times(angle.in(Radians) / Constants.Drive.GEAR_RATIO / 2.0);
  }

  private static AngularVelocity driveLinearVelocityToMotorAngularVelocity(LinearVelocity velocity) {
    return RadiansPerSecond.of(
        velocity.in(MetersPerSecond) * Constants.Drive.GEAR_RATIO * 2.0 / Constants.Drive.WHEEL_DIAMETER.in(Meter)
    );
  }

  private static LinearVelocity motorAngularVelocityToDriveLinearVelocity(AngularVelocity velocity) {
    double wheelRadiansPerSecond = velocity.in(RadiansPerSecond) / Constants.Drive.GEAR_RATIO;
    double wheelLinearVelocity = wheelRadiansPerSecond * (Constants.Drive.WHEEL_DIAMETER.in(Meter) / 2.0);
    return MetersPerSecond.of(wheelLinearVelocity);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        motorAngularVelocityToDriveLinearVelocity(motorLeft.getVelocity().getValue()).in(MetersPerSecond),
        motorAngularVelocityToDriveLinearVelocity(motorRight.getVelocity().getValue()).in(MetersPerSecond)
    );
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  private void configure() {
    config.Slot0.kP = Constants.PID.driveMotorPID.kP.get();
    config.Slot0.kI = Constants.PID.driveMotorPID.kI.get();
    config.Slot0.kD = Constants.PID.driveMotorPID.kD.get();
    config.Slot0.kS = Constants.PID.driveMotorPID.kS.get();
    config.Slot0.kV = Constants.PID.driveMotorPID.kV.get();
    config.Slot0.kA = Constants.PID.driveMotorPID.kA.get();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Limits.Chassis.DRIVE_STATOR_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Limits.Chassis.DRIVE_SUPPLY_CURRENT_LIMIT;


    motorLeft.getConfigurator().apply(config);
    motorLeft.getConfigurator().apply(
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
    );
    motorRight.getConfigurator().apply(config);
    motorRight.getConfigurator().apply(
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
    );
  }

  private void reconfigure() {
    if (
        Constants.PID.driveMotorPID.kP.hasChanged()
            || Constants.PID.driveMotorPID.kI.hasChanged()
            || Constants.PID.driveMotorPID.kD.hasChanged()
            || Constants.PID.driveMotorPID.kS.hasChanged()
            || Constants.PID.driveMotorPID.kV.hasChanged()
            || Constants.PID.driveMotorPID.kA.hasChanged()
    ) {
      System.out.println("Reconfiguring...");

      config.Slot0.kP = Constants.PID.driveMotorPID.kP.get();
      config.Slot0.kI = Constants.PID.driveMotorPID.kI.get();
      config.Slot0.kD = Constants.PID.driveMotorPID.kD.get();
      config.Slot0.kS = Constants.PID.driveMotorPID.kS.get();
      config.Slot0.kV = Constants.PID.driveMotorPID.kV.get();
      config.Slot0.kA = Constants.PID.driveMotorPID.kA.get();

      motorLeft.getConfigurator().apply(config);
      motorLeft.getConfigurator().apply(
          new MotorOutputConfigs()
              .withInverted(InvertedValue.CounterClockwise_Positive)
              .withNeutralMode(NeutralModeValue.Coast)
      );
      motorRight.getConfigurator().apply(config);
      motorRight.getConfigurator().apply(
          new MotorOutputConfigs()
              .withInverted(InvertedValue.Clockwise_Positive)
              .withNeutralMode(NeutralModeValue.Coast)
      );
    }
  }

  public void runTwist(ChassisSpeeds chassisSpeedsDes) {
    DifferentialDriveWheelSpeeds wheelSpeedsDes = kinematics.toWheelSpeeds(chassisSpeedsDes);
    motorLeft.setControl(leftVelocityRequest.withVelocity(
        driveLinearVelocityToMotorAngularVelocity(MetersPerSecond.of(wheelSpeedsDes.leftMetersPerSecond))
    ));
    motorRight.setControl(rightVelocityRequest.withVelocity(
        driveLinearVelocityToMotorAngularVelocity(MetersPerSecond.of(wheelSpeedsDes.rightMetersPerSecond))
    ));

    // logging
    var wheelsSpeedCurr = getWheelSpeeds();
    var chassisSpeedsCurr = getChassisSpeeds();
    Logger.recordOutput(TAG + "/ChassisSpeedsDes", chassisSpeedsDes);
    Logger.recordOutput(TAG + "/ChassisSpeedCurr", chassisSpeedsCurr);
    Logger.recordOutput(TAG + "/WheelSpeedsDes", wheelSpeedsDes);
    Logger.recordOutput(TAG + "/WheelSpeedsCurr", wheelsSpeedCurr);
  }

  @Override
  public void periodic() {
    // odometry update
    poseEstimator.update(
//        gyro.getRotation2d(),
        new Rotation2d(),
        motorAngleToDrivePosition(motorLeft.getPosition().getValue()).in(Meters),
        motorAngleToDrivePosition(motorRight.getPosition().getValue()).in(Meters)
    );

    // dynamic reconfigure
    if (Constants.TUNING)
      reconfigure();
  }
}
