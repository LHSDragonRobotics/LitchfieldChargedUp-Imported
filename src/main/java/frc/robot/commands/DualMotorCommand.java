package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DualMotorController;

public class DualMotorCommand extends Command implements Runnable {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  protected final DualMotorController m_subsystem;
  private double commandPower;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DualMotorCommand(DualMotorController subsystem_param, double power_param) {
    m_subsystem = subsystem_param;
    commandPower = power_param;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void run() {
    execute();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     m_subsystem.go(commandPower*1-RobotContainer.m_driverController.getRightTriggerAxis());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.go(0);
  } 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
