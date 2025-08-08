package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Volts;

public class ShootCommand extends Command {

    private ShooterSubsystem shooterSubsystem;

    public ShootCommand(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.setVoltage(Volts.of(RobotConstants.ShooterConstants.shootVoltage.get()));
    }

    @Override
    public void execute(){

    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.setVoltage(Volts.of(0));
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
