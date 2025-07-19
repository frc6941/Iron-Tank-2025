package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.intake.IntakeSubsystem.SystemState;
import frc.robot.subsystems.intake.IntakeSubsystem.WantedState;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
//  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final Drive drive = new Drive();
  private final IntakeSubsystem intakeSubsystem;
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  // Creates the Xbox controller to drive the robot
  CommandXboxController mainController = new CommandXboxController(0);

  private boolean climberReadyToClimb = false;

  public RobotContainer() {
    if (RobotBase.isReal()) {
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
    
    drive.setDefaultCommand(Commands.run(() -> {
      double v1 = -mainController.getLeftY();
      double v2 = -mainController.getRightX();

      double vX = v1 * 5.0; // theoretical max speed: 5.676 mps
      double omega = Math.signum(v2) * v2 * v2 * 10.0;
      drive.runTwist(new ChassisSpeeds(vX, 0.0, omega));
    }, drive));
    
    // B button - Move pivot back to home position
    mainController.b().onTrue(intakeSubsystem.runOnce(() -> {
      System.out.println("=== B BUTTON PRESSED - Moving to zero position ===");
      intakeSubsystem.setWantedState(WantedState.HOME);;
    }));

    // Left Bumper (LB) - Elevate pivot
    mainController.leftBumper().onTrue(intakeSubsystem.runOnce(() -> {
      System.out.println("=== LEFT BUMPER PRESSED - Elevating pivot ===");
      intakeSubsystem.setWantedState(WantedState.DEPLOY_WITHOUT_ROLL);
    }));

    // Right Trigger (RT) - Eject while held
    mainController.rightTrigger().whileTrue(intakeSubsystem.run(() -> {
      System.out.println("=== RT HELD - Ejecting ===");
      intakeSubsystem.setWantedState(WantedState.HOLD_SHOOT);
    }));
    mainController.rightTrigger().onFalse(intakeSubsystem.runOnce(() -> {
      System.out.println("=== RT RELEASED - Stopping eject ===");
      intakeSubsystem.setWantedState(WantedState.HOLD);
    }));

    // Right Bumper (RB) - Toggle intake (first press: start, second: stop)
    mainController.rightBumper().toggleOnTrue(intakeSubsystem.runOnce(() -> {
      System.out.println("=== RIGHT BUMPER PRESSED - Toggling intake ===");
      if (intakeSubsystem.getSystemState() == SystemState.DEPLOY_INTAKING) {
        intakeSubsystem.setWantedState(WantedState.DEPLOY_WITHOUT_ROLL);
      } else {
        intakeSubsystem.setWantedState(WantedState.DEPLOY_INTAKE);
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
//    return autoChooser.getSelected();
      return Commands.none();
  }

}