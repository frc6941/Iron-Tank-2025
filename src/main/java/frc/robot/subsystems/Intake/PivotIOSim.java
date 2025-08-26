package frc.robot.subsystems.Intake;

import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Rotations;

public class PivotIOSim implements PivotIO {
    private Angle position = Rotations.of(0);

    public PivotIOSim() {

    }

    @Override
    public void setPosition(Angle Position) {
        this.position = Position;
        return;
    }

    @Override
    public void updateConfigs(double kp, double ki, double kd, double ka, double kv, double ks, double kg) {
        return;
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.velocityRotPerSec = 0.0;
        inputs.currentPositionRot = position.in(Rotations);
        inputs.appliedVolts = 0.0;
        inputs.statorCurrentAmps = 0.0;
        inputs.supplyCurrentAmps = 0.0;
        inputs.tempCelsius = 0.0;
    }

    @Override
    public void setVoltage(double volts) {

    }
}

