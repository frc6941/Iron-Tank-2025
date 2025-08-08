package frc.robot.subsystems.tank;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RotationsPerSecond;

public class TankIOReal implements TankIO {
    TalonFX motorRight = new TalonFX(1, "rio");
    private final StatusSignal<AngularVelocity> rightMotorVelocityRotPerSec = motorRight.getVelocity();
    private final StatusSignal<Voltage> rightMotorAppliedVolts = motorRight.getSupplyVoltage();
    private final StatusSignal<Current> rightMotorStatorCurrentAmps = motorRight.getStatorCurrent();
    private final StatusSignal<Current> rightMotorSupplyCurrentAmps = motorRight.getSupplyCurrent();
    private final StatusSignal<Temperature> rightMotorTempCelsius = motorRight.getDeviceTemp();
    TalonFX motorLeft = new TalonFX(0, "rio");
    private final StatusSignal<AngularVelocity> leftMotorVelocityRotPerSec = motorLeft.getVelocity();
    private final StatusSignal<Voltage> leftMotorAppliedVolts = motorLeft.getSupplyVoltage();
    private final StatusSignal<Current> leftMotorStatorCurrentAmps = motorLeft.getStatorCurrent();
    private final StatusSignal<Current> leftMotorSupplyCurrentAmps = motorLeft.getSupplyCurrent();
    private final StatusSignal<Temperature> leftMotorTempCelsius = motorLeft.getDeviceTemp();
    TalonFXConfigurator motorRightConfigurator = motorRight.getConfigurator();
    TalonFXConfigurator motorLeftConfigurator = motorLeft.getConfigurator();

    public TankIOReal() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        motorLeftConfigurator.apply(motorOutputConfigs);
        motorRightConfigurator.apply(motorOutputConfigs);
    }

    public void setRPS(AngularVelocity leftRPS, AngularVelocity rightRPS) {
        Logger.recordOutput("DriveSubsystem/TargetleftRPS", leftRPS.in(RotationsPerSecond));
        Logger.recordOutput("DriveSubsystem/TargetrightRPS", -rightRPS.in(RotationsPerSecond));

        motorLeft.setControl(new VelocityVoltage(leftRPS.in(RotationsPerSecond)));
        motorRight.setControl(new VelocityVoltage(-rightRPS.in(RotationsPerSecond)));
    }

    @Override
    public void updateInputs(TankIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                leftMotorAppliedVolts,
                leftMotorStatorCurrentAmps,
                leftMotorSupplyCurrentAmps,
                leftMotorTempCelsius,
                leftMotorVelocityRotPerSec,
                rightMotorAppliedVolts,
                rightMotorStatorCurrentAmps,
                rightMotorSupplyCurrentAmps,
                rightMotorTempCelsius,
                rightMotorVelocityRotPerSec
        );
        inputs.leftMotorAppliedVolts = leftMotorAppliedVolts.getValueAsDouble();
        inputs.leftMotorStatorCurrentAmps = leftMotorStatorCurrentAmps.getValueAsDouble();
        inputs.leftMotorSupplyCurrentAmps = leftMotorSupplyCurrentAmps.getValueAsDouble();
        inputs.leftMotorTempCelsius = leftMotorTempCelsius.getValueAsDouble();
        inputs.leftMotorVelocityRotPerSec = leftMotorVelocityRotPerSec.getValueAsDouble();
        inputs.rightMotorAppliedVolts = rightMotorAppliedVolts.getValueAsDouble();
        inputs.rightMotorStatorCurrentAmps = rightMotorStatorCurrentAmps.getValueAsDouble();
        inputs.rightMotorSupplyCurrentAmps = rightMotorSupplyCurrentAmps.getValueAsDouble();
        inputs.rightMotorTempCelsius = rightMotorTempCelsius.getValueAsDouble();
        inputs.rightMotorVelocityRotPerSec = rightMotorVelocityRotPerSec.getValueAsDouble();
        if (RobotConstants.TUNING) {
            motorLeft.getConfigurator().apply(new Slot0Configs().withKP(RobotConstants.TankConstants.TANK_PID.kP.get())
                    .withKI(RobotConstants.TankConstants.TANK_PID.kI.get())
                    .withKD(RobotConstants.TankConstants.TANK_PID.kD.get()));
            motorRight.getConfigurator().apply(new Slot0Configs()
                    .withKP(RobotConstants.TankConstants.TANK_PID.kP.get())
                            .withKI(RobotConstants.TankConstants.TANK_PID.kI.get())
                            .withKD(RobotConstants.TankConstants.TANK_PID.kD.get()));
        }
    }
}