package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  // Creates the Xbox controller to drive the robot
  CommandXboxController mainController = new CommandXboxController(0);  

  public RobotContainer() {
    configureBindings();
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
    mainController.a().onTrue(climberSubsystem.runOnce(() -> {
      System.out.println("=== A BUTTON PRESSED - Move climber to zero position ===");
      climberSubsystem.goToZeroPosition();
    }));
    mainController.x().onTrue(climberSubsystem.runOnce(() -> {
      System.out.println("=== X BUTTON PRESSED - Move climber to start position ===");
      climberSubsystem.goToStartPosition();
    }));
    mainController.y().onTrue(climberSubsystem.runOnce(() -> {
      System.out.println("=== Y BUTTON PRESSED - Move climber to stop position ===");
      climberSubsystem.goToStopPosition();
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
    return new SequentialCommandGroup(
      // Move forward for 3 seconds (arcade drive) and move pivot to eject position at the same time
      new ParallelCommandGroup(
        new RunCommand(() -> driveSubsystem.arcadeDrive(0.5, 0.0), driveSubsystem).withTimeout(2.5),
        intakeSubsystem.runOnce(() -> intakeSubsystem.goToEjectPosition())
      ),
      // Stop
      new RunCommand(() -> driveSubsystem.arcadeDrive(0.0, 0.0), driveSubsystem).withTimeout(0.1),
      // Eject for 2 seconds
      new RunCommand(() -> intakeSubsystem.startEject(), intakeSubsystem).withTimeout(1.0),
      // Stop eject
      intakeSubsystem.runOnce(() -> intakeSubsystem.stopEject())
    );
  }
}