package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.consts.PID.pivotPID;

public interface IntakePivotIO {
    default void updateInputs(IntakePivotIOInputs inputs) {
    }

    default void setMotorVoltage(double voltage) {
    }

    default void setPivotAngle(double targetAngleDeg) {
    }

    default void resetAngle(double resetAngleDeg) {
    }

    default boolean isNearAngle(double targetAngleDeg, double toleranceDeg) {
        return false;
    }

    @AutoLog
    class IntakePivotIOInputs {
        public double targetAngleDeg = 0.0;
        public double currentAngleDeg = 0.0;
        public double velocityRotPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double motorVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;

        public double intakePivotKP = pivotPID.kP.get();
        public double intakePivotKI = pivotPID.kI.get();
        public double intakePivotKD = pivotPID.kD.get();
    }
}