package frc.robot.commands.autocmd;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ShootCommand extends Command {
    double startTime;
    double rate;

    public ShootCommand(double rate) {
       this.rate = rate;
       addRequirements(RobotContainer.thrower);       
       addRequirements(RobotContainer.sucker);
    }
    @Override
    public void initialize() { 
        System.out.println("Shooting!");
    }
    @Override
    public void execute() {
        RobotContainer.thrower.go(rate);
        Timer.delay(.1);
        RobotContainer.sucker.go(-1);
    }

// Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {} 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Timer.delay(1);
    RobotContainer.thrower.go(0);
    RobotContainer.sucker.go(0);
    System.out.println("Finished shooting");
    return true;
  }

}
