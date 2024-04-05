package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BasicController;

public class ArmCommand extends BasicCommand {
    double power = 0;
    private XboxController xbox = RobotContainer.m_driverController;
    public static final DigitalInput limit0 = RobotContainer.limit0;
    public static final DigitalInput limit1 = RobotContainer.limit1;
    public boolean limitOverride = false;
    public double timeStarted;
    public int holdingFor = 0;
    double zeroPos = 0;
    double lowerLimit = -5000;
    double pos = 0;
    double armSpeed = .8;
    public MoveGoal goal;
    public boolean isAutonomous = false;

    public ArmCommand(BasicController subsystem_param, double power_param) {
        super(subsystem_param, power_param);
        timeStarted = Timer.getFPGATimestamp();
        goal = MoveGoal.NONE;
        // TODO Auto-generated constructor stub
    }

    public ArmCommand(BasicController subsystem_param, double power_param, MoveGoal newGoal) {
        super(subsystem_param, power_param);
        goal = newGoal;
        isAutonomous = true;
        timeStarted = Timer.getFPGATimestamp();
        // TODO Auto-generated constructor stub
    }

    @Override
    public void initialize() {
        timeStarted = Timer.getFPGATimestamp();
        System.out.println("auto arm! " + goal.nameString);
    }

    public boolean shouldContinue(boolean top) {
        if (Timer.getFPGATimestamp() - timeStarted > 2.25 && goal != MoveGoal.NONE) {
            m_subsystem.go(0);
            System.out.println(
                    "WARNING!!! PROGRAM FORCEFULLY STOPPED ARM BECAUSE IT TOOK TO LONG!! VERIFY INTEGRITY OF LIMIT SWITCHES OR ARM COULD BE SERIOUSLY DAMAGED!!");
            return false;
        }
        if (top) {
            return (limit0.get());
        } else {
            return (limit1.get());
        }
    }

    public void stopGoal() {
        goal = MoveGoal.NONE;
    }

    @Override
    public void execute() {
        // System.out.println(goal.nameString);
        if (xbox.getAButton() && xbox.getYButton()) {
            holdingFor++;
            System.out.println("SAFE HOTKEY PRESSED");
            if (holdingFor > 20) {
                System.out.println("LIMIT SWITCH DISABLED!!!");
            }
        }
        if (!limit0.get()) {
            zeroPos = pos;
        }
        if (xbox.getLeftStickButton()) {
            goal = MoveGoal.BOTTOM;
            timeStarted = Timer.getFPGATimestamp();
        }
        if (xbox.getRightStickButton()) {
            goal = MoveGoal.TOP;
            timeStarted = Timer.getFPGATimestamp();
        }
        if (xbox.getPOV() == 0 || xbox.getPOV() == 180) {
           armSpeed = .5d; 
        } else {
            armSpeed = 1d;
        }
        if ((xbox.getPOV() == 0 || goal.equals(MoveGoal.TOP)) && shouldContinue(true)) {
            power = armSpeed;
        } else if ((xbox.getPOV() == 180 || goal.equals(MoveGoal.BOTTOM)) && shouldContinue(false)) {
            power = -armSpeed;
        } else {
            power = 0;
        }
        if (!isAutonomous) {
            if (goal == MoveGoal.TOP) {
                if (!shouldContinue(true)) {
                    stopGoal();
                    System.out.println("stop up!");
                }
            }
            if (goal == MoveGoal.BOTTOM) {
                if (!shouldContinue(false)) {
                    stopGoal();
                    System.out.println("stop up!");
                }
            }
        }


        m_subsystem.go(power);
    }

    @Override
    public boolean isFinished() {
        if (isAutonomous) {
                if (goal == MoveGoal.TOP) {
                if (!shouldContinue(true)) {
                    m_subsystem.go(0);
                    return true;        
                }
            }
            if (goal == MoveGoal.BOTTOM) {
                if (!shouldContinue(false)) {
                    m_subsystem.go(0);
                    return true;
                }
            }
        }
        return false;
    }

    public enum MoveGoal {
        TOP("top"),
        BOTTOM("bottom"),
        NONE("none");

        MoveGoal(String string) {
            this.nameString = string;
        }

        public String nameString;
    }
}
