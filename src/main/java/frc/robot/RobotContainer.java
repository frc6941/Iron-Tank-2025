package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  // Creates the Xbox controller to drive the robot
  CommandXboxController mainController = new CommandXboxController(0);  

  private boolean climberReadyToClimb = false;

  public RobotContainer() {
    configureBindings();
    autoChooser.addOption("Left", AutoBuilder.buildAuto("Left"));
    autoChooser.addOption("Mid", AutoBuilder.buildAuto("Mid"));
    autoChooser.addOption("Right", AutoBuilder.buildAuto("Right"));
  }

  private void configureBindings() {
    
    // *** MODIFIED: Default drive command now uses Cheesy Drive ***
    driveSubsystem.setDefaultCommand(new RunCommand(() -> {
        // Get joystick values
        // Left stick Y for forward/backward movement
        double forward = -mainController.getLeftY();

        // Right stick X for turning, inverted as in the original code
        double rotation = -mainController.getRightX();

        // Determine if quick turn should be active.
        // This is true if the driver is not commanding significant forward/backward speed.
        boolean isQuickTurn = Math.abs(forward) < consts.Drive.QUICK_TURN_THRESHOLD;

        // Apply the inputs to the new Cheesy Drive method in the subsystem
        driveSubsystem.cheesyDrive(forward, rotation, isQuickTurn);

      }, driveSubsystem));
    
    // ... (No changes to any other bindings for Intake or Climber)

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

    // Left Trigger (LT) - Elevate pivot
    mainController.leftTrigger().onTrue(intakeSubsystem.runOnce(() -> {
      System.out.println("=== LT PRESSED - Elevating pivot ===");
      intakeSubsystem.goToEjectPosition();
    }));

    // Right Trigger (RT) - Eject while held
    mainController.rightTrigger().whileTrue(intakeSubsystem.run(() -> {
      System.out.println("=== RT HELD - Ejecting ===");
      intakeSubsystem.startEject();
    }));
    mainController.rightTrigger().onFalse(intakeSubsystem.runOnce(() -> {
      System.out.println("=== RT RELEASED - Stopping eject ===");
      intakeSubsystem.stopEject();
    }));

    // Right Bumper (RB) - Toggle intake (first press: start, second: stop)
    mainController.rightBumper().toggleOnTrue(intakeSubsystem.runOnce(() -> {
      System.out.println("=== RIGHT BUMPER PRESSED - Toggling intake ===");
      if (!intakeSubsystem.isIntaking()) {
        intakeSubsystem.startIntake();
      } else {
        intakeSubsystem.stopIntake();
      }
    }));

    // Climber Controls
    mainController.x().onTrue(climberSubsystem.runOnce(() -> {
      if (!climberSubsystem.isAtStartPosition()) {
        System.out.println("=== X BUTTON PRESSED - Move to start climb position ===");
        climberSubsystem.goToStartClimb();
      } else {
        System.out.println("=== X BUTTON PRESSED - Start climbing for 5 seconds ===");
        climberSubsystem.startClimbFor5Sec();
      }
    }));
    mainController.a().onTrue(climberSubsystem.runOnce(() -> {
      System.out.println("=== A BUTTON PRESSED - Move to zero position ===");
      climberSubsystem.goToZero();
    }));
  }

  // This deadband method is no longer used for drive, but kept for potential other uses
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