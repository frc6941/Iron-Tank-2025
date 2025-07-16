// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  // Creates the Xbox controller to drive the robot
  CommandXboxController mainController = new CommandXboxController(0);  

  // State variable to track if intake is in eject position
  private boolean isInEjectPosition = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // Build an auto chooser. This will use Commands.none() as the default option.
    final SendableChooser autoChooser = new SendableChooser<>();
    autoChooser.addOption("Left", AutoBuilder.buildAuto("Left"));
    autoChooser.addOption("Mid", AutoBuilder.buildAuto("Mid"));
    autoChooser.addOption("Right", AutoBuilder.buildAuto("Right"));

    SmartDashboard.putData("Auto Chooser", autoChooser);
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
        driveSubsystem.arcadeDrive(forwardInput, -turnInput);
      });

    driveSubsystem.setDefaultCommand(tankDrive);

    // Intake Controls

    // B button - Move pivot back to zero position
    mainController.b().onTrue(intakeSubsystem.runOnce(() -> {
      System.out.println("=== B BUTTON PRESSED - Moving to zero position ===");
      intakeSubsystem.goToZero();
      isInEjectPosition = false; // Reset eject position state when going to zero
    }));

    // Left Bumper (LB) - Elevate pivot
    mainController.leftBumper().onTrue(intakeSubsystem.runOnce(() -> {
      System.out.println("=== LEFT BUMPER PRESSED - Elevating pivot ===");
      intakeSubsystem.elevate();
      isInEjectPosition = false; // Reset eject position state when elevating
    }));

    // Right Bumper (RB) - Intake while held
    mainController.rightBumper().whileTrue(intakeSubsystem.run(() -> {
      System.out.println("=== RIGHT BUMPER HELD - Intaking ===");
      intakeSubsystem.startIntake();
    }));
    mainController.rightBumper().onFalse(intakeSubsystem.runOnce(() -> {
      System.out.println("=== RIGHT BUMPER RELEASED - Stopping intake ===");
      intakeSubsystem.stopIntake();
    }));

    // Left Trigger (LT) - Two-stage behavior: first press moves to eject position, second press starts ejecting
    mainController.leftTrigger().onTrue(intakeSubsystem.runOnce(() -> {
      if (!isInEjectPosition) {
        // First press: move to eject position
        System.out.println("=== LT PRESSED - Moving to eject position ===");
        intakeSubsystem.goToEjectPosition();
        isInEjectPosition = true;
      } else {
        // Second press: start ejecting
        System.out.println("=== LT PRESSED - Starting eject ===");
        intakeSubsystem.startEject();
      }
    }));

    // Stop eject on LT release
    mainController.leftTrigger().onFalse(intakeSubsystem.runOnce(() -> {
      System.out.println("=== LT RELEASED - Stopping eject ===");
      intakeSubsystem.stopEject();
    }));

    // Right Trigger (RT) - Stop roller
    mainController.rightTrigger().onTrue(intakeSubsystem.runOnce(() -> {
      System.out.println("=== RIGHT TRIGGER PRESSED - Stopping roller ===");
      intakeSubsystem.stopRoller();
    }));

    // Climber Controls
    mainController.x().onTrue(climberSubsystem.runOnce(() -> {
      System.out.println("=== X BUTTON PRESSED - Start climbing ===");
      climberSubsystem.setClimberVoltage();
    }));
    mainController.a().onTrue(climberSubsystem.runOnce(() -> {
      System.out.println("=== A BUTTON PRESSED - Stop climbing ===");
      climberSubsystem.stopClimb();
    }));
  }

  public static double deadBand(double value, double tolerance) {
    if (value < tolerance && value > -tolerance) {
      return 0;
    }
    return value;
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
