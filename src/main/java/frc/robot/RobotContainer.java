// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Roller.RollerIO;
import frc.robot.subsystems.Roller.RollerIOReal;
import frc.robot.subsystems.Roller.RollerIOSim;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.tank.TankIOReal;
import frc.robot.subsystems.tank.TankIOSim;
import frc.robot.subsystems.tank.TankSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Creates the Xbox controller to drive the robot
    CommandXboxController mainController = new CommandXboxController(0);
    // The robot's subsystems and commands are defined here...
    private TankSubsystem m_tankSubsystem;
    private ShooterSubsystem m_shooterSubsystem;
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureSubsystems();
        configureBindings();
    }

    // Deadband command to eliminate drifting
    public static double deadBand(double value, double tolerance) {
        return Math.abs(value) > tolerance ? value : 0;
        /*
        If you can't understand the code above, it is equivalent to the following lines:
        if (value < tolerance && value > -tolerance) {
            return 0;
        } else {
            return value;
        }
        */
    }

    private void configureSubsystems() {
        if (RobotBase.isSimulation()) {
            m_tankSubsystem = new TankSubsystem(new TankIOSim());
            m_shooterSubsystem = new ShooterSubsystem(new RollerIOSim(1,2,
                    new SimpleMotorFeedforward(
                            RobotConstants.ShooterConstants.ShooterPID.kS.get(),
                            RobotConstants.ShooterConstants.ShooterPID.kV.get()),
                    new ProfiledPIDController(
                            RobotConstants.ShooterConstants.ShooterPID.kP.get(),
                            RobotConstants.ShooterConstants.ShooterPID.kI.get(),
                            RobotConstants.ShooterConstants.ShooterPID.kD.get(),
                            new TrapezoidProfile.Constraints(15,1)
                    ))); {
            }
        } else {
            m_tankSubsystem = new TankSubsystem(new TankIOReal());
            m_shooterSubsystem = new ShooterSubsystem(new RollerIOReal(
                    7,"rio",100,100,false,false));
        }
    }

    /**
     * Use this method to define your trigger->command mappings.
     */
    private void configureBindings() {
        // Put any trigger->command mappings here.

        // Run motor with setSpeeds command
        Command arcadeDrive =
                m_tankSubsystem.run(
                        () -> {
                            m_tankSubsystem.setArcadeSpeed(
                                    RobotConstants.TankConstants.MAX_SPEED.times(deadBand(-mainController.getLeftY(),0.05)),
                                    RobotConstants.TankConstants.MAX_ANGULAR_SPEED.times(deadBand(-mainController.getRightX(),0.05))
                            );
                        }
                );

        mainController.a().whileTrue(new ShootCommand(m_shooterSubsystem));

        m_tankSubsystem.setDefaultCommand(arcadeDrive);
    }


    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return new Command() {
        };
    }
}