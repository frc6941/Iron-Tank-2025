package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.consts;

public class IntakeSubsystem extends SubsystemBase {
    // Motors
    private static final TalonFX motorPivot = new TalonFX(consts.CANID.PIVOTMOTOR);
    private static final TalonFX motorRoller = new TalonFX(consts.CANID.ROLLERMOTOR);

    // Motor Controls
    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(false);

    // Filter
    private static final LinearFilter filter = LinearFilter.movingAverage(5);

    // Intake states
    public enum WantedState {ZERO, INTAKE, ELEVATE, EJECT, HOLD, HOME, OFF}
    public enum SystemState {ZEROING, INTAKING, ELEVATING, EJECTING, HOLDING, HOMEING, OFF}
    
    // State variables
    private WantedState wantedState = WantedState.ZERO;
    private SystemState systemState = SystemState.ZEROING;
    private boolean hasHomed = false; // Whether the intake has been homed or not
    private double currentFilterValue = 0.0; // Current value for the filter
    private double pivotPosition = 0.0; // Position in rotations
    private double rollerSpeed = 0.0; // Speed in rps


    public IntakeSubsystem() {
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = consts.Limits.Intake.PIVOT_SUPPLY_CURRENT_LIMIT;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.StatorCurrentLimit = consts.Limits.Intake.PIVOT_STATOR_CURRENT_LIMIT;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        pivotConfig.withSlot0(new Slot0Configs()
        .withKP(consts.PID.intakePivotPID.kP.get())
        .withKI(consts.PID.intakePivotPID.kI.get())
        .withKD(consts.PID.intakePivotPID.kD.get()));

        motorPivot.getConfigurator().apply(pivotConfig);

        motorPivot.clearStickyFaults();

        motorPivot.optimizeBusUtilization();
    }

    public void setPivotPosition(double position) {

    }

    public void setPivotVoltage(double voltage) {
        // Ensure the voltage is within the range of -12 to 12 volts
        voltage = MathUtil.clamp(voltage, -12.0, 12.0);
        motorPivot.setControl(voltageOut.withOutput(voltage));
    }

    public void setRollerSpeed(double rps) {

    }

    public void zero() {
        // Naturally we want to stop the roller when zeroing
        setRollerSpeed(0.0);

        // Find the lowest point using current
        if (currentFilterValue <= 18) {
            setPivotVoltage(0.5);
            setWantedState(WantedState.ZERO);
            hasHomed = true;
        }
        if (currentFilterValue > 18) {
            setPivotVoltage(0);
            // pivot.resetAngle(120); // I don't know what this is from frc6941/2025-Competition-Robot
            setWantedState(WantedState.HOME);
            hasHomed = false;
        }
    }

    public void intake() {

    }

    public void elevate() {
        // By elevate we mean to raise the intaker to L1.
    }

    public void eject() {
        // Eject the coral by running the roller in reverse

    }

    public void hold() {

    }

    public void home() {

    }

    public void off() {

    }

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    @Override
    public void periodic() {
        currentFilterValue = filter.calculate(motorPivot.getStatorCurrent().getValueAsDouble());
    }
}
