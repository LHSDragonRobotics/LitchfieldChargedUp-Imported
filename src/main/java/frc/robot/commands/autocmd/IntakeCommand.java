package frc.robot.commands.autocmd;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class IntakeCommand extends Command {
    double timeStarted;
    double rate;
    boolean reverse = false;
    double timeToStop = 1.25                        ;

    public IntakeCommand() {
       addRequirements(RobotContainer.sucker);
    }
    @Override
    public void initialize() { 
        timeStarted = Timer.getFPGATimestamp();
        System.out.println("Sucking!");
    }
    @Override
    public void execute() {
      if (Timer.getFPGATimestamp() - timeStarted > timeToStop-.3) {
        RobotContainer.sucker.go(0.2);
      } else {
        RobotContainer.sucker.go(-0.8);
      }
    }

    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {} 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotContainer.limit2.get() ||Timer.getFPGATimestamp() - timeStarted > timeToStop;
  }
}
