package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.IntakePivotIOReal;
import frc.robot.subsystems.intake.IntakePivotIOSim;
import frc.robot.subsystems.intake.IntakeRollerIOReal;
import frc.robot.subsystems.intake.IntakeRollerIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem.WantedState;
import frc.robot.subsystems.intake.IntakeSubsystem.WantedState.*;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem;
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  // Creates the Xbox controller to drive the robot
  CommandXboxController mainController = new CommandXboxController(0);  

  private boolean climberReadyToClimb = false;

  public RobotContainer() {
    if (RobotBase.isReal()){
      intakeSubsystem = new IntakeSubsystem(new IntakePivotIOReal(), new IntakeRollerIOReal());
    } else {
      intakeSubsystem = new IntakeSubsystem(new IntakePivotIOSim(), new IntakeRollerIOSim());
    }

    configureBindings();
    autoChooser.addOption("Left", AutoBuilder.buildAuto("Left")); 
    autoChooser.addOption("Mid", AutoBuilder.buildAuto("Mid"));
    autoChooser.addOption("Right", AutoBuilder.buildAuto("Right"));
    SmartDashboard.putData("Chooser", autoChooser);

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
      intakeSubsystem.setWantedState(WantedState.HOME);
    }));

    // Left Bumper (LB) - Elevate pivot
    mainController.a().onTrue(intakeSubsystem.runOnce(() -> {
      System.out.println("=== LEFT BUMPER PRESSED - Elevating pivot ===");
      intakeSubsystem.setWantedState(WantedState.DEPLOY_WITHOUT_ROLL);
    }));

    // Left Trigger (LT)
    // We don't need this anymore - this should be auto-processed
    // mainController.leftTrigger().onTrue(intakeSubsystem.runOnce(() -> {
    //   System.out.println("=== LT PRESSED - Elevating pivot ===");
    //   intakeSubsystem.goToEjectPosition();
    // }));

    // Right Trigger (RT) - Eject while held
    mainController.x().onTrue(intakeSubsystem.runOnce(() -> {
      System.out.println("=== RT HELD - Ejecting ===");
      intakeSubsystem.setWantedState(WantedState.SHOOT);;
    }));
    // This is auto-processed, so we don't need a release command
    // mainController.rightTrigger().onFalse(intakeSubsystem.runOnce(() -> {
    //   System.out.println("=== RT RELEASED - Stopping eject ===");
    //   intakeSubsystem.stopEject();
    // }));

    // Right Bumper (RB) - Start intake, auto stop
    mainController.y().onTrue(intakeSubsystem.runOnce(() -> {
      System.out.println("=== RIGHT BUMPER PRESSED - Toggling intake ===");
      intakeSubsystem.setWantedState(WantedState.DEPLOY_INTAKE);
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