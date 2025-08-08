package frc.robot.subsystems.tank;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.RotationsPerSecond;

public class TankIOSim implements TankIO {
    AngularVelocity leftRPS = RotationsPerSecond.of(0);
    AngularVelocity rightRPS = RotationsPerSecond.of(0);

    @Override
    public void setRPS(AngularVelocity leftRPS, AngularVelocity rightRPS) {
        this.leftRPS = leftRPS;
        this.rightRPS = rightRPS;
        SmartDashboard.putNumber("DriveSubsystem/leftRPS", leftRPS.in(RotationsPerSecond));
        SmartDashboard.putNumber("DriveSubsystem/rightRPS", rightRPS.in(RotationsPerSecond));
    }

    @Override
    public void updateInputs(TankIOInputs inputs) {
        inputs.leftMotorVelocityRotPerSec = leftRPS.in(RotationsPerSecond);
        inputs.rightMotorVelocityRotPerSec = rightRPS.in(RotationsPerSecond);
    }
}