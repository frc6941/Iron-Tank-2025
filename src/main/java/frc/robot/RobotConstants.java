package frc.robot;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.utils.TunableNumber;

import static edu.wpi.first.units.Units.*;

public class RobotConstants {
    public static final boolean TUNING = true;

    public static class TankConstants {
        public static LinearVelocity MAX_SPEED = MetersPerSecond.of(6.0);//m/s
        public static AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(Math.PI * 2);//rad/s
        public static double GEAR_RATIO = 12.0;
        public static Distance WHEEL_TRACK = Meters.of(0.548005);//m
        public static Distance WHEEL_RADIUS = Inches.of(3);


        public static class TANK_PID {
            public static final TunableNumber kP = new TunableNumber("TANK_PID/KP", 0.3);
            public static final TunableNumber kI = new TunableNumber("TANK_PID/KI", 0);
            public static final TunableNumber kD = new TunableNumber("TANK_PID/KD", 0);
        }
    }

    public static class ShooterConstants {

        public static final TunableNumber shootVoltage = new TunableNumber("Shooter/ShootVoltage", -6);

        public static class ShooterPID {
            public static final TunableNumber kP = new TunableNumber("SHOOTER_PID/KP", 0.3);
            public static final TunableNumber kI = new TunableNumber("SHOOTER_PID/KI", 0.0);
            public static final TunableNumber kD = new TunableNumber("SHOOTER_PID/KD", 0.0);
            public static final TunableNumber kS = new TunableNumber("SHOOTER_PID/KS", 0.0);
            public static final TunableNumber kV = new TunableNumber("SHOOTER_PID/KV", 0.0);
            public static final TunableNumber kA = new TunableNumber("SHOOTER_PID/KA", 0.0);
        }
    }

    public static class IntakeConstants {
        public static final int PIVOT_MOTOR_ID = 5;
        public static final int ROLLER_MOTOR_ID = 6;
        public static final int CAN_CODER_ID = 8;
        public static final TunableNumber INTAKE_POSITION_DEGREES = new TunableNumber("Intake/INTAKE_POSITION_DEGREES", -0.037);
        public static final TunableNumber ELEVATED_POSITION_DEGREES = new TunableNumber("Intake/ELEVATE_POSITION_DEGREES", 0.168);
        public static final TunableNumber INTAKE_VOLTAGE = new TunableNumber("Intake/INTAKE_VOLTAGE", 4);
        public static final TunableNumber EJECT_VOLTAGE = new TunableNumber("Intake/EJECT_VOLTAGE", -3);

        public static class IntakePivotPID {
            public static final TunableNumber kP = new TunableNumber("INTAKE_PID/Pivot/KP", 130);
            public static final TunableNumber kI = new TunableNumber("INTAKE_PID/Pivot/KI", 0);
            public static final TunableNumber kD = new TunableNumber("INTAKE_PID/Pivot/KD", 5);
            public static final TunableNumber kS = new TunableNumber("INTAKE_PID/Pivot/KS", 0.0);
            public static final TunableNumber kV = new TunableNumber("INTAKE_PID/Pivot/KV", 0.0);
            public static final TunableNumber kA = new TunableNumber("INTAKE_PID/Pivot/KA", 0.0);
            public static final TunableNumber kG = new TunableNumber("INTAKE_PID/Pivot/KA", 0.0);
        }

        public static class IntakeRollerPID {
            public static final TunableNumber kP = new TunableNumber("INTAKE_PID/Roller/KP", 0.1);
            public static final TunableNumber kI = new TunableNumber("INTAKE_PID/Roller/KI", 0.0);
            public static final TunableNumber kD = new TunableNumber("INTAKE_PID/Roller/KD", 0.0);
            public static final TunableNumber kS = new TunableNumber("INTAKE_PID/Roller/KS", 0.0);
            public static final TunableNumber kV = new TunableNumber("INTAKE_PID/Roller/KV", 0.0);
            public static final TunableNumber kA = new TunableNumber("INTAKE_PID/Roller/KA", 0.0);
        }
    }

}
