package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

public class Diagnostics {
    public static boolean armError;

    public static void updateErrors() {
        SmartDashboard.putBoolean("armOK", !armError);
    }
    public static void updateDashboard() {
        double armPower = RobotContainer.armMotor.getOutputCurrent(); 
        SmartDashboard.putNumber("armPower", armPower);
        SmartDashboard.putNumber("voltage", RobotContainer.armMotor.getBusVoltage());
        SmartDashboard.putNumber("gyro", DriveSubsystem.m_gyro.getAngle());
    }
    public static void armError() {
        armError = true;
    }
}