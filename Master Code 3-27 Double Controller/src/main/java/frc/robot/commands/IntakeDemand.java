package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class IntakeDemand extends CommandBase {
    public IntakeDemand() {
        addRequirements(((Subsystem)Robot.m_shintake));
    }

    @Override
    public void initialize() {
        Robot.m_shintake.setSpeed(0.0);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}

}