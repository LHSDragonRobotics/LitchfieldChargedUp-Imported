package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DualMotorController extends SubsystemBase {

  // Single-motor mechanism....
  private  MotorController motor = null;
  private  MotorController motor2 = null;


  /** Creates a new DriveSubsystem. */
  public DualMotorController(int CANID, int CANID2, boolean kBrushless) {
    if (kBrushless) {
        motor2 = new CANSparkMax(CANID2,  CANSparkLowLevel.MotorType.kBrushless);
        motor = new CANSparkMax(CANID,  CANSparkLowLevel.MotorType.kBrushless);
    } else {
        motor2 = new CANSparkMax(CANID2,  CANSparkLowLevel.MotorType.kBrushed);
        motor = new CANSparkMax(CANID,  CANSparkLowLevel.MotorType.kBrushed);     
    }
  }

  @Override
  public void periodic() {
  }

  /**
   */
  public void go(double power) {
    motor.set(power);
    motor2.set(power);
  }
    public void goReversed(double power) {
    motor.set(power);
    motor2.set(-power);
  }
}