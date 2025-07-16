package frc.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.WantedState;

public class SimpleCoralAuto extends Command {
    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private Timer timer;
    private double drive_seconds = 3.25;
    private double exjest_seconds = 4.5;

    /**
     * This auto will have the robot drive forwards, stop, then drop the coral into L1
     * 
     * There are many ways to write autos, this style will work well for most simple
     * auto routines. For more advanced routines you may want a different structure and 
     * to use more sensors.
     * 
     * Here we use two timer gates, after the robot has finished driving for the first 3.25 
     * seconds, it will exjest the coral for 4.5-3.25 = 1.25 seconds.
     * 
     * 
     * @param drive
     * @param roller
     * @param arm
     */
    public SimpleCoralAuto(DriveSubsystem drive, IntakeSubsystem intake)
    {
        driveSubsystem = drive;
        intakeSubsystem = intake;
        
        timer = new Timer();

        addRequirements(driveSubsystem);
        addRequirements(intakeSubsystem);
    }

    @Override
  public void initialize() {
    // start timer, uses restart to clear the timer as well in case this command has
    // already been run before
    timer.restart();
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    /**
     * While this timer is less than drive_seconds, the robot will obey the command inside
     */
    if(timer.get() < drive_seconds)
    {
        driveSubsystem.driveArcade(0.3, 0.0,false);
        intakeSubsystem.setWantedState(WantedState.ZERO);
    }
    /**
     * Once the timer is greater than drive_seconds but less than exjest seconds,
     * the code inside will run, here we stop the drivetrain and exjest the coral.
     */
    else if(timer.get() > drive_seconds && timer.get() < exjest_seconds)
    {
        driveSubsystem.driveArcade(0.0, 0.0,false);
        intakeSubsystem.setWantedState(WantedState.EJECT);
    }
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
    // stop drive motors
    driveSubsystem.driveArcade(0.0, 0.0, false);
    intakeSubsystem.setWantedState(WantedState.ZERO);;
    timer.stop();
  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    // check if timer exceeds seconds, when it has this will return true indicating
    // this command is finished
    return timer.get() >= exjest_seconds;
  }
}
