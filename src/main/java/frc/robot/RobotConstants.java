package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.utils.TunableNumber;

public class RobotConstants {
    public static final boolean TUNING = true;

    public static class TankConstants{
        public static double MAX_SPEED_METERS_PER_SECOND = 3.5;//m/s
        public static double MAX_ANGULAR_SPEED_RAD_PER_SECOND = Math.PI;//rad/s
        public static double GEAR_RATIO = 12.0;
        public static double WHEEL_TRACK = 0.548005;//m
        public static double WHEEL_RADIUS = Units.inchesToMeters(3);


        public static class TANK_PID{
            public static final TunableNumber kP = new TunableNumber("TANK_PID/KP",0.1);
            public static final TunableNumber kI = new TunableNumber("TANK_PID/KI",0.1);
            public static final TunableNumber kD = new TunableNumber("TANK_PID/KD",0.1);
        }
    }


}
