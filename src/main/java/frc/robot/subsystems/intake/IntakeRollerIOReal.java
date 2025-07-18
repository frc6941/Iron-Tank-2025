package frc.robot.subsystems.intake;

import frc.robot.Constants;
import frc.robot.subsystems.roller.RollerIOReal;

public class IntakeRollerIOReal extends RollerIOReal implements IntakeRollerIO {
    private static final int id = Constants.CANID.MOTOR_ROLLER;
    private static final double statorCurrentLimitAmps = Constants.Limits.Intake.ROLLER_STATOR_CURRENT_LIMIT;
    private static final double supplyCurrentLimitAmps = Constants.Limits.Intake.ROLLER_SUPPLY_CURRENT_LIMIT;
    private static final boolean invert = Constants.Superstructures.Intake.IS_INVERT;
    private static final boolean brake = Constants.Superstructures.Intake.IS_BRAKE;

    public IntakeRollerIOReal() {
        super(id, statorCurrentLimitAmps, supplyCurrentLimitAmps, invert, brake);
    }
}
