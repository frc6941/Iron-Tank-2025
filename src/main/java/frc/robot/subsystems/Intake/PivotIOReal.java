package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.*;
import frc.robot.RobotConstants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class PivotIOReal implements PivotIO {

    private final TalonFX motor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, "rio");
    private final CANcoder canCoder = new CANcoder(IntakeConstants.CAN_CODER_ID, "rio");

    private final StatusSignal<AngularVelocity> velocityRotPerSec = motor.getVelocity();
    private final StatusSignal<Voltage> appliedVolts = motor.getSupplyVoltage();
    private final StatusSignal<Voltage> motorVolts = motor.getMotorVoltage();
    private final StatusSignal<Current> statorCurrentAmps = motor.getStatorCurrent();
    private final StatusSignal<Current> supplyCurrentAmps = motor.getSupplyCurrent();
    private final StatusSignal<Temperature> tempCelsius = motor.getDeviceTemp();
    private final StatusSignal<Angle> currentPositionRot = motor.getPosition();

    public PivotIOReal() {
        configureMotors();
        CANcoderConfiguration CANcoder = new CANcoderConfiguration();
        CANcoder.MagnetSensor.MagnetOffset = -0.230712890625; // Set your desired offset here
        CANcoder.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        canCoder.getConfigurator().apply(CANcoder);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                velocityRotPerSec,
                appliedVolts,
                motorVolts,
                statorCurrentAmps,
                supplyCurrentAmps,
                tempCelsius,
                currentPositionRot
        );
    }

    public void configureMotors() {
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        pivotConfig.Feedback.FeedbackRemoteSensorID = IntakeConstants.CAN_CODER_ID;
        pivotConfig.Feedback.RotorToSensorRatio = (12.0 * 50) / 11;
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        updateConfigs(
                IntakeConstants.IntakePivotPID.kP.get(),
                IntakeConstants.IntakePivotPID.kI.get(),
                IntakeConstants.IntakePivotPID.kD.get(),
                IntakeConstants.IntakePivotPID.kA.get(),
                IntakeConstants.IntakePivotPID.kV.get(),
                IntakeConstants.IntakePivotPID.kS.get(),
                IntakeConstants.IntakePivotPID.kG.get()
        );

        motor.getConfigurator().apply(pivotConfig);
    }

    @Override
    public void updateConfigs(double kp, double ki, double kd, double ka, double kv, double ks, double kg) {
        motor.getConfigurator().apply(new Slot0Configs()
                .withKP(kp)
                .withKI(ki)
                .withKD(kd)
                .withKS(ka)
                .withKV(kv)
                .withKG(ks)
                .withKG(kg));
    }

    @Override
    public void setPosition(Angle Position) {
        motor.setControl(new PositionVoltage(Position));
        Logger.recordOutput("Pivot Position", Position.in(Rotations));
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                velocityRotPerSec,
                appliedVolts,
                motorVolts,
                statorCurrentAmps,
                supplyCurrentAmps,
                tempCelsius,
                currentPositionRot
        );
        inputs.velocityRotPerSec = velocityRotPerSec.getValue().in(RotationsPerSecond);
        inputs.currentPositionRot = currentPositionRot.getValue().in(Rotations);
        inputs.appliedVolts = appliedVolts.getValue().in(Volts);
        inputs.statorCurrentAmps = statorCurrentAmps.getValue().in(Amps);
        inputs.supplyCurrentAmps = supplyCurrentAmps.getValue().in(Amps);
        inputs.tempCelsius = tempCelsius.getValue().in(Celsius);
    }
}
