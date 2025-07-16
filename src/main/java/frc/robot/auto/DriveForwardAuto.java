package frc.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForwardAuto extends Command {
    private DriveSubsystem driveSubsystem;
    private Timer timer;
    private double drive_seconds = 3.25;

      /**
     * This auto will have the robot drive forwards
     * 
     * There are many ways to write autos, this form will work well for most simple
     * auto routines. For more advanced routines you may want a different structure and 
     * to use more sensors.
     * 
     * Here we use a single timer gate, after the robot has finished driving for the first 3.25 
     * seconds it will stop moving. You may wish for the robot to move more or less depending on
     * your use case.
     * 
     * 
     * @param drive
     */
    public DriveForwardAuto(DriveSubsystem drive)
    {
        driveSubsystem = drive;
        
        timer = new Timer();

        addRequirements(driveSubsystem);
    }

    @Override
  public void initialize() {
    // start timer, uses restart to clear the timer as well in case this command has
    // already been run before
    timer.restart();
  }

  // Runs every cycle while the command is scheduled (~50 times per second), here we will just drive forwards
  @Override
  public void execute() {
    // drive at 30% speed
    if(timer.get() < drive_seconds)
    {
        driveSubsystem.driveArcade(0.3, 0.0,false);
    }
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
    // stop drive motors
    driveSubsystem.driveArcade(0.0, 0.0, false);
    timer.stop();
  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    // check if timer exceeds seconds, when it has this will return true indicating
    // this command is finished
    return timer.get() >= drive_seconds;
  }
}
