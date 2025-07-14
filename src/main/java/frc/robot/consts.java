package frc.robot;

import frc.robot.utils.TunableNumber;

/*
 * T0 Constants that are either measured, tuned, or set by the team.
 */
public class consts {

    /* =============================== |
     *             CAN ID              |
     * ============================== */
    public static final class CANID {
        // Chassis Motors
        public static final int LEFTMOTOR = 4;
        public static final int LEFTMOTORFOLLEWER = 2;
        public static final int RIGHTMOTOR = 1;
        public static final int RIGHTMOTORFOLLOWER = 3;

        // Intake Motors
        public static final int PIVOTMOTOR = 5; 
        public static final int ROLLERMOTOR = 6; 
    }

    /* =============================== |
     *               PID               |
     * ============================== */
    public static final class PID {
        public static final class driveMotorPID {
            public static final TunableNumber kP = new TunableNumber("drive_kP", 1);
            public static final TunableNumber kI = new TunableNumber("drive_kI", 0.0);
            public static final TunableNumber kD = new TunableNumber("drive_kD", 0.0);
            public static final TunableNumber kS = new TunableNumber("drive_kS", 0.0);
            public static final TunableNumber kV = new TunableNumber("drive_kV", 0.0);
            public static final TunableNumber kA = new TunableNumber("drive_kA", 0.0);
            public static final TunableNumber kG = new TunableNumber("drive_kG", 0.0);
        }
        public static final class positionPID{
            public static final TunableNumber kP = new TunableNumber("position_kP", 1);
            public static final TunableNumber kI = new TunableNumber("position_kI", 0.0);
            public static final TunableNumber kD = new TunableNumber("position_kD", 0.0);
            public static final TunableNumber kS = new TunableNumber("position_kS", 0.0);
            public static final TunableNumber kV = new TunableNumber("position_kV", 0.0);
            public static final TunableNumber kA = new TunableNumber("position_kA", 0.0);
            public static final TunableNumber kG = new TunableNumber("position_kG", 0.0);
        }
        public static final class pivotPID {
            public static final TunableNumber kP = new TunableNumber("pivot_kP", 1);
            public static final TunableNumber kI = new TunableNumber("pivot_kI", 0.0);
            public static final TunableNumber kD = new TunableNumber("pivot_kD", 0.0);
        }
        public static final class rollerPID {
            public static final TunableNumber kP = new TunableNumber("roller_kP", 1);
            public static final TunableNumber kI = new TunableNumber("roller_kI", 0.0);
            public static final TunableNumber kD = new TunableNumber("roller_kD", 0.0);
        }
    }

    /* =============================== |
     *          Superstructures        |
     * ============================== */
    public static final class Superstructures {
        public static final class Chassis {
            public static final double TRACK_WIDTH = 0.677;
            public static final double WHEEL_DIAMETER = 0.1524;
            public static final double GEAR_RATIO = 8.46; // Motor rotations per wheel rotation TODO: Update to real value
        }

        public static final class Intake {
            public static final double PIVOT_MAX_ANGLE = 90.0; // Maximum angle in degrees
            public static final double PIVOT_MIN_ANGLE = 0.0; // Minimum angle in degrees
            public static final double HOLD_ANGLE_OFFSET = 0.5; // Hold position offset from zero in rotations
            public static final double POSITION_TOLERANCE = 0.05; // Position tolerance in rotations

            public static final double L1 = 100; // Height in degrees for L1 state
        }
    }

    /* =============================== |
     *             Limits              |
     * ============================== */
    public static final class Limits {
        public static final class Chassis {
            public static final double MAX_VELOCITY = 10.0;
            public static final double MAX_OUTPUT = 0.8;
        }

        public static final class Intake {
            // These values are copied from frc6941/2025-Competition-Robot
            public static final double PIVOT_SUPPLY_CURRENT_LIMIT = 40.0; // Supply current limit in Amperes
            public static final double PIVOT_STATOR_CURRENT_LIMIT = 40.0; // Stator current limit in Amperes
            public static final double ROLLER_SUPPLY_CURRENT_LIMIT = 40.0; // Supply current limit in Amperes
            public static final double ROLLER_STATOR_CURRENT_LIMIT = 40.0; // Stator current limit in Amperes
            public static final double ROLLER_INTAKE_SPEED = 20.0; // Roller speed in rotations per second
        }
    }

    /* =============================== |
     *          Miscellaneous          |
     * ============================== */
    // TunableNumber
    public static final boolean TUNING = true;
    
    // Intake tunable constants
    public static final TunableNumber INTAKE_ROLLER_SPEED = new TunableNumber("intake_roller_speed", 20.0);
    public static final TunableNumber INTAKE_HOLD_OFFSET = new TunableNumber("intake_hold_offset", 0.5);
    public static final TunableNumber INTAKE_ELEVATED_POSITION = new TunableNumber("intake_elevated_position", 8);
    public static final TunableNumber INTAKE_POSITION_TOLERANCE = new TunableNumber("intake_position_tolerance", 0.05);
    public static final TunableNumber INTAKE_HOME_POSITION = new TunableNumber("intake_home_position", 0.0);
    public static final TunableNumber INTAKE_PIVOT_SPEED = new TunableNumber("intake_pivot_speed", 0.7);
}
