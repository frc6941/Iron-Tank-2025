package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.TunableNumber;

/*
 * T0 Constants that are either measured, tuned, or set by the team.
 */
public class consts {
    //Robot
    public static final class Chassis {
        public static final double TRACK_WIDTH = 10;
        
    }



    /* =============================== |
     *             CAN ID              |
     * ============================== */
    public static final class CANID {
        // Chassis Motors
        public static final int MOTOR_LEFT = 4;
        public static final int MOTOR_LEFT_FOLLEWER = 2;
        public static final int MOTOR_RIGHT = 1;
        public static final int MOTOR_RIGHT_FOLLOWER = 3;

        // Intake Motors
        public static final int PIVOTMOTOR = 5; 
        public static final int ROLLERMOTOR = 6; 
        public static final int PIVOTENCODER = 8;

        // Climber CAN ID and PID
        public static final int CLIMBER_MOTOR = 7; // Update to your actual CAN ID

        // Gyro (Pigeon 1.0)
        public static final int GYRO = 0;
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
            public static final TunableNumber kP = new TunableNumber("pivot_kP", 4);
            public static final TunableNumber kI = new TunableNumber("pivot_kI", 0.2);
            public static final TunableNumber kD = new TunableNumber("pivot_kD", 0.1);
        }
        public static final class rollerPID {
            public static final TunableNumber kP = new TunableNumber("roller_kP", 3);
            public static final TunableNumber kI = new TunableNumber("roller_kI", 0.0);
            public static final TunableNumber kD = new TunableNumber("roller_kD", 0.0);
        }
        public static final class climberPID {
            public static final TunableNumber kP = new TunableNumber("climber_kP", 1.0);
            public static final TunableNumber kI = new TunableNumber("climber_kI", 0.0);
            public static final TunableNumber kD = new TunableNumber("climber_kD", 0.0);
        }
    }

    /* =============================== |
     *          Superstructures        |
     * ============================== */
    public static final class Superstructures {
        public static final class Chassis {
            public static final double TRACK_WIDTH = 0.677;
            public static final double WHEEL_DIAMETER = 0.1524;
            public static final double GEAR_RATIO = 8.45; // Motor rotations per wheel rotation 
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
            public static final double ROLLER_SUPPLY_CURRENT_LIMIT = 40.0; // Supply current limit in Ampere
            public static final double ROLLER_STATOR_CURRENT_LIMIT = 40.0; // Stator current limit in Amperes
            public static final double ROLLER_INTAKE_SPEED = 30.0; // Roller speed in rotations per second
        }
    }

    /* =============================== |
     *             Controls            |
     * ============================== */
    public static final class Controls {
        public static final class Intake {
            // Has Coral Amperes
            // Copied from frc6941/2025-Competition-Robot, use with caution
            public static final TunableNumber ROLLER_AMPS_HAS_CORAL = new TunableNumber("roller_amps_has_coral", 55.0); 

            // Intake roller voltage
            public static final TunableNumber INTAKE_VOLTAGE = new TunableNumber("intake_voltage", 12.0); // Voltage in Volts
        }
    }

    /* =============================== |
     *          Miscellaneous          |
     * ============================== */
    // TunableNumber
    public static final boolean TUNING = true;
    
    // Intake tunable constants+
    public static final TunableNumber INTAKE_ROLLER_SPEED = new TunableNumber("intake_roller_speed", 30.0);
    public static final TunableNumber INTAKE_HOLD_OFFSET = new TunableNumber("intake_hold_offset", 0.5);
    public static final TunableNumber INTAKE_ELEVATED_POSITION = new TunableNumber("intake_elevated_position", 1.46);
    public static final TunableNumber INTAKE_POSITION_TOLERANCE = new TunableNumber("intake_position_tolerance", 0.05);
    public static final TunableNumber INTAKE_HOME_POSITION = new TunableNumber("intake_home_position", 0.0);
    public static final TunableNumber INTAKE_PIVOT_SPEED = new TunableNumber("intake_pivot_speed", 0.7);
    public static final TunableNumber INTAKE_EJECT_POSITION = new TunableNumber("intake_eject_position", 0.5); // default value, adjust as needed
    // Tunable voltages for pivot and roller
    public static final TunableNumber INTAKE_PIVOT_VOLTAGE = new TunableNumber("intake_pivot_voltage", 4.0); // default value, adjust as needed
    public static final TunableNumber INTAKE_ROLLER_VOLTAGE = new TunableNumber("intake_roller_voltage", 4.0); // default value, adjust as needed
    // Climber tunable constant
    public static final TunableNumber CLIMBER_VOLTAGE = new TunableNumber("climber_voltage", 4.0); // default value, adjust as needed
}
