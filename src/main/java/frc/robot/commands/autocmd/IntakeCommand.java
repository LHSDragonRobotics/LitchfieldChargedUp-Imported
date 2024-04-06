package frc.robot.commands.autocmd;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class IntakeCommand extends Command {
    double timeStarted;
    double rate = 0.8;
    boolean reverse = false;
    double timeToStop = 1.25                        ;

    public IntakeCommand() {
       addRequirements(RobotContainer.sucker);
    }
    public IntakeCommand(double time, double rate) {
      timeToStop = time;
      reverse = true;
      this.rate = rate;
      addRequirements(RobotContainer.sucker);
    }
    @Override
    public void initialize() { 
        timeStarted = Timer.getFPGATimestamp();
        System.out.println("Sucking!");
    }
    @Override
    public void execute() {
      if (Timer.getFPGATimestamp() - timeStarted > timeToStop-.1 && !reverse) {
        RobotContainer.sucker.go(-0.3);
      } else {
        RobotContainer.sucker.go(rate);
      }
    }

    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {} 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    int range = RobotContainer.colorSensor.getProximity();
    SmartDashboard.putNumber("sensorRange", range);
    return (range < 5) ||Timer.getFPGATimestamp() - timeStarted > timeToStop;
  }
}
