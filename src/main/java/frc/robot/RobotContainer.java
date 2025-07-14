// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // Creates the Xbox controller to drive the robot
  CommandXboxController mainController = new CommandXboxController(0);  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /** Use this method to define your trigger->command mappings. */
  private void configureBindings() {
    // Put any trigger->command mappings here.
    Command tankDrive =
      driveSubsystem.run(() -> {
        // Left stick Y for forward/backward movement
        double forwardInput = deadBand(-mainController.getLeftY(), 0.1);

        // Right stick X for turning (left/right)
        double turnInput = deadBand(mainController.getRightX(), 0.1);

        // Apply the inputs to the drive subsystem
        driveSubsystem.arcadeDrive(forwardInput, turnInput);
      });

    driveSubsystem.setDefaultCommand(tankDrive);

    // Intake Controls
    // Y button - Set current position as zero
    mainController.y().onTrue(intakeSubsystem.runOnce(() -> {
      System.out.println("=== Y BUTTON PRESSED - Setting zero position ===");
      intakeSubsystem.setZeroPosition();
    }));

    // B button - Move pivot back to zero position
    mainController.b().onTrue(intakeSubsystem.runOnce(() -> {
      System.out.println("=== B BUTTON PRESSED - Moving to zero position ===");
      intakeSubsystem.goToZero();
    }));

    // Left Bumper (LB) - Elevate pivot
    mainController.leftBumper().onTrue(intakeSubsystem.runOnce(() -> {
      System.out.println("=== LEFT BUMPER PRESSED - Elevating pivot ===");
      intakeSubsystem.elevate();
    }));

    // Right Bumper (RB) - Intake clockwise
    mainController.rightBumper().onTrue(intakeSubsystem.runOnce(() -> {
      System.out.println("=== RIGHT BUMPER PRESSED - Starting intake (clockwise) ===");
      intakeSubsystem.startIntake();
    }));

    // Left Trigger (LT) - Eject counterclockwise
    mainController.leftTrigger().onTrue(intakeSubsystem.runOnce(() -> {
      System.out.println("=== LEFT TRIGGER PRESSED - Starting eject (counterclockwise) ===");
      intakeSubsystem.startEject();
    }));

    // Right Trigger (RT) - Stop roller
    mainController.rightTrigger().onTrue(intakeSubsystem.runOnce(() -> {
      System.out.println("=== RIGHT TRIGGER PRESSED - Stopping roller ===");
      intakeSubsystem.stopRoller();
    }));
  }

  public static double deadBand(double value, double tolerance) {
    if (value < tolerance && value > -tolerance) {
      return 0;
    }
    return value;
  }
  
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Command() {};
  }
}
