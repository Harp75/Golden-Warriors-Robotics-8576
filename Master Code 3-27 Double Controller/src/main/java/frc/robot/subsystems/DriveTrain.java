package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.manualDrive;



public class DriveTrain extends SubsystemBase{
  private static CANSparkMaxLowLevel motorLeft1 = new CANSparkMax(RobotContainer.leftMaster, MotorType.kBrushless);
  private static CANSparkMaxLowLevel motorLeft2 = new CANSparkMax(RobotContainer.leftFollower, MotorType.kBrushless);
  private static CANSparkMaxLowLevel motorRight1 = new CANSparkMax(RobotContainer.rightMaster, MotorType.kBrushless);
  private static CANSparkMaxLowLevel motorRight2 = new CANSparkMax(RobotContainer.rightFollower, MotorType.kBrushless);

  MotorControllerGroup leftMotorGroup = new MotorControllerGroup(motorLeft1, motorLeft2);
  MotorControllerGroup rightMotorGroup = new MotorControllerGroup(motorRight1, motorRight2);

  public DifferentialDrive drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  public void manualDrive(double move, double turn) {
    drive.arcadeDrive(move, turn);
  }

  public void moveDriveTrain(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
  }
  
  //push
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
      new manualDrive();
  }
}
