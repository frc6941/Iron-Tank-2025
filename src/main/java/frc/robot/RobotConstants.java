package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.utils.TunableNumber;

import static edu.wpi.first.units.Units.*;

public class RobotConstants {
    public static final boolean TUNING = true;

    public static class TankConstants{
        public static LinearVelocity MAX_SPEED = MetersPerSecond.of(3.5);//m/s
        public static AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(Math.PI);//rad/s
        public static double GEAR_RATIO = 12.0;
        public static Distance WHEEL_TRACK = Meters.of(0.548005);//m
        public static Distance WHEEL_RADIUS = Inches.of(3);


        public static class TANK_PID {
            public static final TunableNumber kP = new TunableNumber("TANK_PID/KP", 0.1);
            public static final TunableNumber kI = new TunableNumber("TANK_PID/KI", 0.1);
            public static final TunableNumber kD = new TunableNumber("TANK_PID/KD", 0.1);
        }
    }

    public static class ShooterConstants{

        public static final TunableNumber shootVoltage = new TunableNumber("Shooter/ShootVoltage", 6);


        public static class ShooterPID {
            public static final TunableNumber kP = new TunableNumber("SHOOTER_PID/KP", 0.1);
            public static final TunableNumber kI = new TunableNumber("SHOOTER_PID/KI", 0.0);
            public static final TunableNumber kD = new TunableNumber("SHOOTER_PID/KD", 0.0);
            public static final TunableNumber kS = new TunableNumber("SHOOTER_PID/KS", 0.0);
            public static final TunableNumber kV = new TunableNumber("SHOOTER_PID/KV", 0.0);
            public static final TunableNumber kA = new TunableNumber("SHOOTER_PID/KA", 0.0);
        }
    }

}
