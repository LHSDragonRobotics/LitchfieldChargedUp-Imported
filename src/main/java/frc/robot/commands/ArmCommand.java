package frc.robot.commands;


import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Diagnostics;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BasicController;

public class ArmCommand extends BasicCommand {
    double power = 0;
    private XboxController xbox = RobotContainer.m_driverController;
    private XboxController xbox2 = RobotContainer.m_driverController2;
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
    public static final CANSparkMax armMotor = RobotContainer.armMotor;
    double armPower = 0;
    

    public ArmCommand(BasicController subsystem_param, double power_param) {
        super(subsystem_param, power_param);
        timeStarted = Timer.getFPGATimestamp();
        goal = MoveGoal.NONE;
    }

    public ArmCommand(BasicController subsystem_param, double power_param, MoveGoal newGoal) {
        super(subsystem_param, power_param);
        goal = newGoal;
        isAutonomous = true;
        timeStarted = Timer.getFPGATimestamp();
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
                    Diagnostics.armError();
                return false;
        }
        if (top) {
            return (limit0.get());
        } else {
            return (limit1.get() || xbox.getPOV() == 180 && armPower > 40);
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
                Diagnostics.armError();
            }
        }
        if (!limit0.get()) {
            zeroPos = pos;
        }
        if (xbox.getLeftStickButton()) {
            //.goal = MoveGoal.BOTTOM;
            timeStarted = Timer.getFPGATimestamp();
        }
        if (xbox.getRightStickButton()) {
            goal = MoveGoal.TOP;
            timeStarted = Timer.getFPGATimestamp();
        }
        if (xbox.getPOV() == 0 || xbox.getPOV() == 180 || xbox2.getPOV() == 0 || xbox2.getPOV() == 180) {
           armSpeed = .5d; 
        } else {
            armSpeed = 1d;
        }
        if ((xbox.getPOV() == 0 || xbox2.getPOV() == 0 || goal.equals(MoveGoal.TOP)) && shouldContinue(true)) {
            power = armSpeed;
        } else if ((xbox.getPOV() == 180 || xbox2.getPOV() == 180 || goal.equals(MoveGoal.BOTTOM)) && shouldContinue(false)) {
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
                    System.out.println("stop down!");
                }
            }
        }


        double armPower = armMotor.getOutputCurrent(); 
        if (armPower > 40) {
            System.out.println("WARNING: armPower exceded limit! Value was "+armPower);
            Diagnostics.armError();
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
