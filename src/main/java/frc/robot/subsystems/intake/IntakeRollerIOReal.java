package frc.robot.subsystems.intake;

import frc.robot.consts;
import frc.robot.subsystems.roller.RollerIOReal;

public class IntakeRollerIOReal extends RollerIOReal implements IntakeRollerIO {
    private static final int id = consts.CANID.MOTOR_ROLLER;
    private static final double statorCurrentLimitAmps = consts.Limits.Intake.ROLLER_STATOR_CURRENT_LIMIT;
    private static final double supplyCurrentLimitAmps = consts.Limits.Intake.ROLLER_SUPPLY_CURRENT_LIMIT;
    private static final boolean invert = consts.Superstructures.Intake.IS_INVERT;
    private static final boolean brake = consts.Superstructures.Intake.IS_BRAKE;

    public IntakeRollerIOReal() {
        super(id, statorCurrentLimitAmps, supplyCurrentLimitAmps, invert, brake);
    }
}
