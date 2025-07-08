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
        public static final int LEFTMOTOR = 0;
        public static final int LEFTMOTORFOLLEWER = 1;
        public static final int RIGHTMOTOR = 2;
        public static final int RIGHTMOTORFOLLOWER = 3;
    }

    /* =============================== |
     *               PID               |
     * ============================== */
    public static final class PID {
        public static final class driveMotorPID {
            public static final TunableNumber kP = new TunableNumber("kP", 0.0);
            public static final TunableNumber kI = new TunableNumber("kI", 0.0);
            public static final TunableNumber kD = new TunableNumber("kD", 0.0);
            public static final TunableNumber kS = new TunableNumber("kS", 0.0);
            public static final TunableNumber kV = new TunableNumber("kV", 0.0);
            public static final TunableNumber kA = new TunableNumber("kA", 0.0);
            public static final TunableNumber kG = new TunableNumber("kG", 0.0);
        }
    }

    /* =============================== |
     *          Motion Magic           |
     * ============================== */
    public static final class MotionMagic {
        public static final class leftMotorMM {
            public static final TunableNumber CRUISE_VELOCITY = new TunableNumber("leftCruiseVelocity", 1000.0);
            public static final TunableNumber ACCELERATION = new TunableNumber("leftAcceleration", 500.0);
            public static final TunableNumber JERK = new TunableNumber("leftJerk", 100.0);
        }
        public static final class rightMotorMM {
            public static final TunableNumber CRUISE_VELOCITY = new TunableNumber("rightCruiseVelocity", 1000.0);
            public static final TunableNumber ACCELERATION = new TunableNumber("rightAcceleration", 500.0);
            public static final TunableNumber JERK = new TunableNumber("rightJerk", 100.0);
        }
    }

    /* =============================== |
     *          Superstructures        |
     * ============================== */
    public static final class Superstructures {
        public static final class Chassis {
            public static final double TRACK_WIDTH = 0.69; // TODO: Update to real value
            public static final double WHEEL_DIAMETER = 0.1524; // TODO: Update to real value
            public static final double GEAR_RATIO = 10.71; // Motor rotations per wheel rotation TODO: Update to real value
        }
    }

    /* =============================== |
     *             Limits              |
     * ============================== */
    public static final class Limits {
        public static final double MAX_VELOCITY = 1.0;
        public static final double MAX_OUTPUT = 0.2;
    }

    /* =============================== |
     *          Miscellaneous          |
     * ============================== */
    // TunableNumber
    public static final boolean TUNING = true;
}
