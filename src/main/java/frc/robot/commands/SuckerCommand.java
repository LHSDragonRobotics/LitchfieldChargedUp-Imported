package frc.robot.commands;

import frc.robot.subsystems.BasicController;

public class SuckerCommand extends BasicCommand {
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