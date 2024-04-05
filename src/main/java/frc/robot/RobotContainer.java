// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmCommand;
//import frc.robot.commands.ArmTeleCommand;
import frc.robot.commands.BasicCommand;
import frc.robot.commands.DualMotorCommand;
import frc.robot.commands.RobotDrive;
import frc.robot.commands.SuckerCommand;
import frc.robot.commands.ArmCommand.MoveGoal;
import frc.robot.commands.autocmd.IntakeCommand;
import frc.robot.commands.autocmd.ShootCommand;
import frc.robot.subsystems.BasicController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DualMotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // Motor controller commands
  //public static final BasicController arm = new BasicController(9);
  //public static final BasicController claw = new BasicController(11);
  public static final BasicController sucker = new BasicController(10,false);
  public static final DualMotorController thrower = new DualMotorController(11, 12, true);
  public static final BasicController arm = new BasicController(13,false);

  public static final DigitalInput limit0 = new DigitalInput(0);
  public static final DigitalInput limit1 = new DigitalInput(1);  
  public static final DigitalInput limit2 = new DigitalInput(2);

  //public static final WPI_TalonSRX armEncoder = new WPI_TalonSRX(14);

  // The driver's controller
  public static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public static boolean followPathPlanner = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    NamedCommands.registerCommand("shoot", new ShootCommand(-1)/*.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)*/);
    NamedCommands.registerCommand("armDown", new ArmCommand(arm,0,MoveGoal.BOTTOM));
    NamedCommands.registerCommand("armUp", new ArmCommand(arm,0,MoveGoal.TOP));    
    NamedCommands.registerCommand("intake", new IntakeCommand()/*.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)*/);


    
    SmartDashboard.putString("Auto Selector", "path");
    SmartDashboard.putBoolean("SwapSide", false);
    SmartDashboard.putBoolean("isAuto", DriverStation.isAutonomous());
    SmartDashboard.putBoolean("isTele", !DriverStation.isTeleopEnabled());

    //arm.setDefaultCommand(new ArmTeleCommand());
    //claw.setDefaultCommand(new BasicCommand(claw, .1));
    sucker.setDefaultCommand(new SuckerCommand(sucker, 0d));
    thrower.setDefaultCommand(new DualMotorCommand(thrower, 0d));
    arm.setDefaultCommand(new ArmCommand(arm, 0d));
    // Configure the button bindings
    configureButtonBindings();

boolean USE_REV_CMD = false;

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        
        (USE_REV_CMD) ?
              new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    false, true),
                m_robotDrive)
        : 
              new RobotDrive()
            );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
        new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                  .whileTrue(new DualMotorCommand(thrower, .3d));
        new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                  .whileTrue(new SuckerCommand(sucker, -.8d));
        new JoystickButton(m_driverController, XboxController.Button.kX.value)
                  .whileTrue(new BasicCommand(sucker, .5d));
        new JoystickButton(m_driverController, XboxController.Button.kY.value)
                  .whileTrue(new DualMotorCommand(thrower, -1d));
        new JoystickButton(m_driverController, XboxController.Button.kY.value)
                  .whileTrue(new SuckerCommand(sucker, -.4d));
      /*   WPI sample-code..........
         new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
            */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
    AutoBuilder.buildAuto(SmartDashboard.getString("Auto Selector","test"))
    );
    //return new SequentialCommandGroup(new ShootCommand(1), new AutoDrive(1));
        // Autos.exampleAuto();
  }
  
  
  public Command UNUSED_EXAMPLE_getAutonomousCommand() {

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}