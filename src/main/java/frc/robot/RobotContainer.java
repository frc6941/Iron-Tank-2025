// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  // Creates the Xbox controller to drive the robot
  CommandXboxController mainController = new CommandXboxController(0);  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    
    // Debug: Check controller connection
    System.out.println("Checking for controllers...");
    System.out.println("Controller 0 connected: " + mainController.getHID().isConnected());
    System.out.println("Controller 0 name: " + mainController.getHID().getName());
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
        
        // Log controller inputs for debugging
        System.out.println("Forward: " + forwardInput + ", Turn: " + turnInput);
      });

    driveSubsystem.setDefaultCommand(tankDrive);
    
    // Test command - Press A button to turn 90 degrees right
    new JoystickButton(mainController.getHID(), 1).onTrue(
      driveSubsystem.runOnce(() -> {
        System.out.println("Testing turn command - turning 90 degrees right");
        driveSubsystem.turnInPlace(90.0);
      })
    );
    
    // Test command - Press B button to turn 90 degrees left
    new JoystickButton(mainController.getHID(), 2).onTrue(
      driveSubsystem.runOnce(() -> {
        System.out.println("Testing turn command - turning 90 degrees left");
        driveSubsystem.turnInPlace(-90.0);
      })
    );
    
    // Test command - Press X button to reset facing angle
    new JoystickButton(mainController.getHID(), 3).onTrue(
      driveSubsystem.runOnce(() -> {
        System.out.println("Resetting facing angle");
        driveSubsystem.resetFacingAngle();
      })
    );
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
