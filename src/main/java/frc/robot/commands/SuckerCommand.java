package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BasicController;
import frc.robot.subsystems.DualMotorController;

public class SuckerCommand extends BasicCommand {
    private XboxController xbox = RobotContainer.m_driverController;
    double suckerPower = 0d;
    public SuckerCommand(BasicController subsystem_param, double power_param) {
        super(subsystem_param, power_param);
        suckerPower = power_param;
    }
     @Override
     public void execute() {

        m_subsystem.go(suckerPower);
     }
}