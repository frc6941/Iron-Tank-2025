package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import frc.robot.utils.TunableNumber;

/*
 * T0 Constants that are either measured, tuned, or set by the team.
 */
public class consts {

    /* =============================== |
     *             CAN ID              |
     * ============================== */
    public static final class CANID {
        // ... (no changes here)
        public static final int MOTOR_LEFT = 4;
        public static final int MOTOR_LEFT_FOLLEWER = 2;
        public static final int MOTOR_RIGHT = 1;
        public static final int MOTOR_RIGHT_FOLLOWER = 3;
        public static final int MOTOR_PIVOT = 5; 
        public static final int MOTOR_ROLLER = 6; 
        public static final int ENCODER_PIVOT = 8;
        public static final int MOTOR_CLIMBER = 7;
        public static final int GYRO = 0;

    }

    /* =============================== |
     *               PID               |
     * ============================== */
    public static final class PID {
        // ... (no changes here)
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
     *             Joysticks           |
     * ============================== */
    // *** NEW: Constants for the Cheesy Drive implementation ***
    public static final class Drive {
        // Deadband for the joysticks. Any input below this value will be treated as zero.
        public static final double DEADBAND = 0.05;
        
        // A threshold for when the robot is considered "slow" enough to allow turning in place.
        public static final double QUICK_TURN_THRESHOLD = 0.2;

        // Constants for the negative inertia (steering inertia counteraction)
        public static final class CheesyDrive {
            // How much to scale the corrective power when stopping a turn. Start tuning this value.
            public static final double NEG_INERTIA_TURN_SCALAR = 4.0; 

            // How much to scale the corrective power when the robot is moving straight but still turning slowly.
            public static final double NEG_INERTia_CLOSE_SCALAR = 3.0;

            // How much to scale the corrective power when the driver is actively continuing a turn.
            public static final double NEG_INERTIA_SCALAR = 2.0;

            // Any turn input below this is considered "not turning" for the purposes of inertia calculation.
            public static final double NEG_INERTIA_THRESHOLD = 0.8;
        }
    }


    /* =============================== |
     *          Superstructures        |
     * ============================== */
    public static final class Superstructures {
        // ... (no changes here)
        public static final class Chassis {
            public static final double TRACK_WIDTH = 0.677;
            public static final double WHEEL_DIAMETER = 0.1524;
            public static final double GEAR_RATIO = 8.45; // Motor rotations per wheel rotation
            
            
            // Calculated values based on the above constants
            public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // in meters
            public static final double METERS_PER_ROTATION = WHEEL_CIRCUMFERENCE / GEAR_RATIO;
            public static final double ROTATIONS_PER_METER = 1.0 / METERS_PER_ROTATION;
        }

        public static final class Intake {
            public static final double PIVOT_MAX_ANGLE = 90.0; // Maximum angle in degrees
            public static final double PIVOT_MIN_ANGLE = 0.0; // Minimum angle in degrees
            public static final double HOLD_ANGLE_OFFSET = 0.5; // Hold position offset from zero in rotations
            public static final double POSITION_TOLERANCE = 0.05; // Position tolerance in rotations

            public static final double L1 = 100; // Height in degrees for L1 state

            // Minimum velocity to consider if motorRoller has stopped from a coral
            public static final TunableNumber HAS_CORAL_VELOCITY_THRESHOLD = new TunableNumber("has_coral_velocity_threshold", 1.0);
            // Maximum stator current to consider if motorRoller has stopped from a coral
            public static final TunableNumber HAS_CORAL_CURRENT_THRESHOLD = new TunableNumber("has_coral_current_threshold", 40.0);
            // Eject time in seconds
            public static final TunableNumber EJECT_TIME = new TunableNumber("eject_time", 2.0);
        }
    }

    /* =============================== |
     *             Sensors             |
     * ============================== */
    public static final class Sensors {
        public static final class Encoder {
            public static final double INTAKE_CANCODER_MAGET_OFFSET = -0.382080078125;
            public static final double ROTOR_TO_SENSOR_RATIO = 12.0;
        }
    }

    /* =============================== |
     *             Limits              |
     * ============================== */
    public static final class Limits {
        // ... (no changes here)
        public static final class Chassis {
            public static final double MAX_VELOCITY = 1.0;
            public static final double MAX_OUTPUT = 0.8;
        }

        public static final class Intake {
            public static final double PIVOT_SUPPLY_CURRENT_LIMIT = 40.0;
            public static final double PIVOT_STATOR_CURRENT_LIMIT = 40.0;
            public static final double ROLLER_SUPPLY_CURRENT_LIMIT = 40.0;
            public static final double ROLLER_STATOR_CURRENT_LIMIT = 40.0;
            public static final double ROLLER_INTAKE_SPEED = 30.0;
        }
    }

    /* =============================== |
     *          Miscellaneous          |
     * ============================== */
    // ... (no changes here)
    public static final boolean TUNING = true;
    public static final TunableNumber INTAKE_ROLLER_POWER = new TunableNumber("intake_roller_power", 1.0);
    public static final TunableNumber INTAKE_HOLD_OFFSET = new TunableNumber("intake_hold_offset", 0.5);
    public static final TunableNumber INTAKE_ELEVATED_POSITION = new TunableNumber("intake_elevated_position", 1.46);
    public static final TunableNumber INTAKE_POSITION_TOLERANCE = new TunableNumber("intake_position_tolerance", 0.05);
    public static final TunableNumber INTAKE_HOME_POSITION = new TunableNumber("intake_home_position", 0.0);
    public static final TunableNumber INTAKE_PIVOT_SPEED = new TunableNumber("intake_pivot_speed", 0.7);
    public static final TunableNumber INTAKE_EJECT_POSITION = new TunableNumber("intake_eject_position", 0.5);
    public static final TunableNumber INTAKE_PIVOT_VOLTAGE = new TunableNumber("intake_pivot_voltage", 4.0);
    public static final TunableNumber INTAKE_ROLLER_VOLTAGE = new TunableNumber("intake_roller_voltage", 4.0);
    public static final TunableNumber CLIMBER_VOLTAGE = new TunableNumber("climber_voltage", 4.0);
    public static final TunableNumber CLIMBER_START_POSITION = new TunableNumber("climber_start_position", -18.0); // Set default as needed
    public static final TunableNumber CLIMBER_ZERO_POSITION = new TunableNumber("climber_zero_position", -22.632812); // Set default as needed
}
