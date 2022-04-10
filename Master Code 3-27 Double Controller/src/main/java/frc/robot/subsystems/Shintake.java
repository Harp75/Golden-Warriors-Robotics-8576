package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeDemand;


public class Shintake extends SubsystemBase{
    private VictorSPX motor1 = new WPI_VictorSPX(RobotContainer.Motor_Intake_1);
    //private VictorSPX motor2 = new WPI_VictorSPX(RobotContainer.Motor_Intake_2);

    public void initDefaultCommand() {
       new IntakeDemand();
    }


    public void setSpeed(double speed) {
        motor1.set(ControlMode.PercentOutput, speed);
     //   motor2.set(ControlMode.PercentOutput, speed);
    }

    public void setMotor1(double Speed) {
        motor1.set(ControlMode.PercentOutput, Speed);
    }

    /*public void setMotor2(double Speed) {
        motor2.set(ControlMode.PercentOutput, Speed);
    }*/
}