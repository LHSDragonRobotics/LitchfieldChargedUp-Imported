// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;


import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoDrive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private static NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();

    //private double rate = 0.3;
    double xrate;
    double zRate;
    double yrate;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoDrive(double rate_param) {


    m_subsystem = RobotContainer.m_robotDrive ;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Timer timer = new Timer();
    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override 
  public void execute() {
    xrate = 0;
    zRate = 0;
    yrate = -.5;
    lockOnTag(.8f);
    m_subsystem.drive(-yrate, -xrate, -zRate, false, true);
  }
  public void lockOnTag(float rotrate) {
    double tx = tableInstance.getTable("limelight").getEntry("tx").getDouble(0);
    double ta = tableInstance.getTable("limelight").getEntry("ta").getDouble(0);
    //double tagID = tableInstance.getTable("limelight").getEntry("tid").getDouble(0);
    double[] trans = new double[3];
    trans = tableInstance.getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
    if (tx == 0.0) {
      return;
    }
    xrate = ((trans[4])/-80)*rotrate;
    zRate = tx/80;
    yrate = ((ta-1.5)/2)*rotrate;
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {} 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
