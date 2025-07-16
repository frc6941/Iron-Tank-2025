package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
    default void updateInputs(RollerIOInputs inputs) {
    }

    default void updateConfigs(double kp, double ki, double kd, double ka, double kv, double ks) {
    }

    default void setVoltage(double voltage) {
    }

    default void stop() {
        setVoltage(0.0);
    }

    default void setVelocity(double velocityRPS) {
    }

    @AutoLog
    class RollerIOInputs {
        public double velocityRotPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }
}
