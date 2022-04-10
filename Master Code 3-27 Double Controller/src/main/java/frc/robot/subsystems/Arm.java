package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
//import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeDemand;

public class Arm extends SubsystemBase{  
    //private static CANSparkMax motor3 = new CANSparkMax(RobotContainer.Motor_Arm_1, MotorType.kBrushless);
    private static VictorSPX motor3 = new WPI_VictorSPX(RobotContainer.Motor_Arm_1);

    public void initDefaultCommand() {
        new IntakeDemand();
    }

    public void setSpeed(double speed) {
        //motor3.set(speed);
        motor3.setNeutralMode(NeutralMode.Brake);
        motor3.set(ControlMode.PercentOutput, speed);
    }
}
