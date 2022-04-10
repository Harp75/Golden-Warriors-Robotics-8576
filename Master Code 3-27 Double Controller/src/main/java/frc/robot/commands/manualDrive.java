package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class manualDrive extends CommandBase {
    public manualDrive() {
      addRequirements((Subsystem) Robot.m_Drivetrain);
    }
    
      // Called just before this Command runs the first time
      public void initialize() {
      };
    
      // Called repeatedly when this Command is scheduled to run
      public void execute() {
        double move =  -1 * Robot.m_OI.j.getY();
        double turn = Robot.m_OI.j.getX(); // could be Z
        Robot.m_Drivetrain.manualDrive(move, turn);
        
      }
    
      // Make this return true when this Command no longer needs to run execute()
      public boolean isFinished() {
        return false;
      }
    
      // Called once after isFinished returns true
      protected void end() {}
    
      // Called when another command which requires one or more of the same
      // subsystems is scheduled to run
      protected void interrupted() {}
}