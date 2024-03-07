package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
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

    public ArmCommand(BasicController subsystem_param, double power_param) {
        super(subsystem_param, power_param);
        //TODO Auto-generated constructor stub
    }
    
    @Override
    public void execute() { 
        if (xbox.getAButton() && xbox.getYButton()) {
            holdingFor++;
            System.out.println("SAFE HOTKEY PRESSED");
            if (holdingFor > 20) {
                limitOverride = true;
                System.out.println("LIMIT SWITCH DISABLED!!!");
            }
        }
        if ((xbox.getPOV() == 0 || xbox.getRightStickButton()) && (limit0.get() || limitOverride)) {
            power = .5d;
        } else if ((xbox.getPOV() == 180 || xbox.getLeftStickButton()) && (limit1.get() || limitOverride)) {
            power = -.5d;
        } else {
            power = 0;
        }
        m_subsystem.go(power);
    }
}
