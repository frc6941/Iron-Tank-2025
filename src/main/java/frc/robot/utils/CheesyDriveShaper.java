package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.DoubleSupplier;

public class CheesyDriveShaper {
  private final DoubleSupplier throttleSupplier;
  private final DoubleSupplier rotationSupplier;

  private final TunableNumber maxLinearVelocity = new TunableNumber("CheesyDrive/MaxLinearVelocity", 4.5);
  private final TunableNumber maxAngularVelocity = new TunableNumber("CheesyDrive/MaxAngularVelocity", 7.0);
  private final TunableNumber maxAngularAcceleration = new TunableNumber("CheesyDrive/MaxAngularAcceleration", 20.0);
  private final TunableNumber deadband = new TunableNumber("CheesyDrive/Deadband", 0.05);
  private final TunableNumber negInertiaScalar = new TunableNumber("CheesyDrive/NegInertiaScalar", 3.0);
  private final TunableNumber negInertiaThreshold = new TunableNumber("CheesyDrive/NegInertiaThreshold", 0.65);
  private final TunableNumber negInertiaTurnScalar = new TunableNumber("CheesyDrive/NegInertiaTurnScalar", 4.0);
  private final TunableNumber negInertiaCloseScalar = new TunableNumber("CheesyDrive/NegInertiaCloseScalar", 2.5);

  private double oldWheel = 0.0;
  private double negInertiaAccumulator = 0.0;

  private double prevAngularVelocity = 0.0;
  private double lastTimestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

  public CheesyDriveShaper(DoubleSupplier throttleSupplier, DoubleSupplier rotationSupplier) {
    this.throttleSupplier = throttleSupplier;
    this.rotationSupplier = rotationSupplier;
  }

  public ChassisSpeeds getChassisSpeeds(boolean isQuickTurn) {
    double throttle = applyDeadband(throttleSupplier.getAsDouble());
    double wheel = applyDeadband(rotationSupplier.getAsDouble());

    // Negative inertia logic
    double negInertia = wheel - oldWheel;
    oldWheel = wheel;

    double inertiaScalar;
    if (wheel * negInertia > 0) {
      inertiaScalar = negInertiaScalar.get();
    } else if (Math.abs(wheel) > negInertiaThreshold.get()) {
      inertiaScalar = negInertiaTurnScalar.get();
    } else {
      inertiaScalar = negInertiaCloseScalar.get();
    }

    double negInertiaPower = negInertia * inertiaScalar;
    negInertiaAccumulator += negInertiaPower;

    wheel += negInertiaAccumulator;

    // Decay the accumulator
    if (negInertiaAccumulator > 1.0) {
      negInertiaAccumulator -= 1.0;
    } else if (negInertiaAccumulator < -1.0) {
      negInertiaAccumulator += 1.0;
    } else {
      negInertiaAccumulator = 0.0;
    }

    // Angular velocity computation
    double desiredAngularVel;
    if (isQuickTurn) {
      desiredAngularVel = wheel * maxAngularVelocity.get();
    } else {
      desiredAngularVel = Math.abs(throttle) * wheel * maxAngularVelocity.get();
    }

    // Limit angular acceleration
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    double dt = now - lastTimestamp;
    lastTimestamp = now;

    double maxDelta = maxAngularAcceleration.get() * dt;
    double delta = desiredAngularVel - prevAngularVelocity;
    delta = MathUtil.clamp(delta, -maxDelta, maxDelta);

    double finalAngularVel = prevAngularVelocity + delta;
    prevAngularVelocity = finalAngularVel;

    return new ChassisSpeeds(throttle * maxLinearVelocity.get(), 0.0, finalAngularVel);
  }

  private double applyDeadband(double value) {
    return Math.abs(value) < deadband.get() ? 0.0 : value;
  }
}
