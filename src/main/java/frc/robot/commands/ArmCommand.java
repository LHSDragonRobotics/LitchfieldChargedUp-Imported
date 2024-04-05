package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BasicController;

public class ArmCommand extends BasicCommand {
    double power = 0;
    private XboxController xbox = RobotContainer.m_driverController;
    public static final DigitalInput limit0 = RobotContainer.limit0;
    public static final DigitalInput limit1 = RobotContainer.limit1;
    public static boolean limitOverride = false;
    public int holdingFor = 0;
    double zeroPos = 0;
    double lowerLimit = -5000;
    double pos = 0;
    public MoveGoal goal = MoveGoal.NONE;
    public boolean stopWhenHit = false;

    public ArmCommand(BasicController subsystem_param, double power_param) {
        super(subsystem_param, power_param);
        addRequirements(subsystem_param);
        //TODO Auto-generated constructor stub
    }
    public ArmCommand(BasicController subsystem_param, double power_param, MoveGoal goal) {
        super(subsystem_param, power_param);
        this.goal = goal;
        stopWhenHit = true;
        //TODO Auto-generated constructor stub
    }

    public boolean shouldContinue(boolean top) {
        if (top) {
            return (limit0.get() || limitOverride);
        } else {
            return ((limit1.get() && pos > lowerLimit+zeroPos) || limitOverride);
        }
    }
    public void stopGoal() {
        goal = MoveGoal.NONE;
    }
    
    @Override
    public void execute() { 
        //System.out.println(zeroPos+" "+pos); 
        if (xbox.getAButton() && xbox.getYButton()) {
            holdingFor++;
            System.out.println("SAFE HOTKEY PRESSED");
            if (holdingFor > 20) {
                limitOverride = true;
                System.out.println("LIMIT SWITCH DISABLED!!!");
            }
        }
        if (!limit0.get()) {
            zeroPos = pos;
        }
        if (xbox.getLeftStickButton()) {
            goal = MoveGoal.BOTTOM;
        }
        if (xbox.getRightStickButton()) {
            goal = MoveGoal.TOP;
        }
        if ((xbox.getPOV() == 0 || goal.equals(MoveGoal.TOP)) && shouldContinue(true) ) {
            power = .5d;
        } else if ((xbox.getPOV() == 180 || goal.equals(MoveGoal.BOTTOM)) && shouldContinue(false)) {
            power = -.5d;
        } else {
            power = 0;
        }
        if (goal == MoveGoal.TOP) {
            if (!shouldContinue(true)) {
                stopGoal();
            }
        }
        if (goal == MoveGoal.BOTTOM) {
            if (!shouldContinue(false)) {
                stopGoal();
            }
        }
        m_subsystem.go(power);
    }
      @Override
  public boolean isFinished() {
    if (stopWhenHit) {
       if (goal == MoveGoal.NONE) {
          return true;
       }
    }
    return false;
    }
    public enum MoveGoal {
        TOP,
        BOTTOM,
        NONE;
    }
}
