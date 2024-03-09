package frc.robot.commands.autocmd;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class IntakeCommand extends Command {
    double startTime;
    double rate;

    public IntakeCommand() {
       addRequirements(RobotContainer.sucker);
    }
    @Override
    public void initialize() { 
        System.out.println("Sucking!");
    }
    @Override
    public void execute() {
        RobotContainer.sucker.go(-.8);
    }

    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {} 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Timer.delay(1);
    RobotContainer.sucker.go(0);
    System.out.println("Finished sucking");
    return true;
  }
}
