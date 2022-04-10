// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
/*import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;*/

import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.commands.ArmMove;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shintake;
//import edu.wpi.first.vision.VisionThread;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static final Shintake m_shintake = new Shintake();
  public static final Arm m_arm = new Arm();
  public static final OI m_OI = new OI();
  public static final DriveTrain m_Drivetrain = new DriveTrain();

  public static VictorSPX motor1 = new VictorSPX(RobotContainer.Motor_Intake_1);
  //public static VictorSPX motor2 = new VictorSPX(RobotContainer.Motor_Intake_2);
  //public static CANSparkMaxLowLevel motor3 = new CANSparkMax(RobotContainer.Motor_Arm_1, MotorType.kBrushless);
  public static VictorSPX motor3 = new VictorSPX(RobotContainer.Motor_Arm_1);

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public static final int IMG_WIDTH = 320;
  public static final int IMG_HEIGHT = 240;

  /*private VisionThread visionThread;
  private double centerX = 0.0;

  private final Object imgLock = new Object();*/


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    //m_autonomousCommand = new MoveSequence();


    CameraServer.startAutomaticCapture();

    /*UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
        if (!pipeline.filterContoursOutput().isEmpty()) {
            Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
            synchronized (imgLock) {
                centerX = r.x + (r.width / 2);
            }
        }
    });
    visionThread.start();*/


  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    //CommandScheduler.getInstance().run();
  }
  double startTime;
  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
   // CommandScheduler.getInstance().run(); //CODE WE ADDED
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    startTime = Timer.getFPGATimestamp();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    //  m_autonomousCommand.start();
    }
    //m_Drivetrain.moveDriveTrain(.3, .3); //LINE CHANGE
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double time = Timer.getFPGATimestamp();
    /*if (time - startTime < 3 && time - startTime > 0) {
      m_Drivetrain.drive.arcadeDrive(0, 0.3);
    }
    else {
      m_Drivetrain.drive.arcadeDrive(0, 0);
    }*/
    if (time - startTime < 4 && time - startTime > 3) {
      //Robot.motor3.set(ControlMode.PercentOutput, 0.2);
      //new ArmMove(0.1);

      Robot.motor1.set(ControlMode.PercentOutput, 0.8);
    }
    else {
      Robot.motor1.set(ControlMode.PercentOutput, 0);
      //Robot.motor3.set(ControlMode.PercentOutput, 0);
    }
    
    if (time - startTime < 8 && time - startTime > 4){
      m_Drivetrain.drive.arcadeDrive(0, -0.5);
    }
    else {
      m_Drivetrain.drive.arcadeDrive(0, 0);
    }
    if (time - startTime < 4 && time - startTime <6) {
      Robot.motor3.set(ControlMode.PercentOutput, -.3);
    }



    
    



    /*double centerX;
    synchronized (imgLock) {
        centerX = this.centerX;
    }
    double turn = centerX - (IMG_WIDTH / 2);
    m_Drivetrain.drive.arcadeDrive(turn * 0.005, 0.4);
    m_shintake.setSpeed(-0.3);*/

    
    /*if(time - startTime < 3){
      m_Drivetrain.drive.arcadeDrive(0, 0.3);
      //m_Drivetrain.moveDriveTrain(-0.2, -0.2);
    }
   else{
      m_Drivetrain.drive.arcadeDrive(0, 0);
    }

    if(time - startTime < 5 && time - startTime > 3){
      //m_arm.setSpeed(0.3);
      new ArmMove(0.3);
      Robot.motor1.set(ControlMode.PercentOutput, 0.3);

    }
    else{
      m_arm.setSpeed(0.0);
    }
    if(time - startTime < 4 && time - startTime > 3){

      m_Drivetrain.drive.arcadeDrive(0.,- 0.2);
    }
    else{
      m_Drivetrain.drive.arcadeDrive(0, 0);
    }
    if(time - startTime < 9 && time - startTime > 7){
     // m_shintake.setSpeed(0.3);

      m_Drivetrain.drive.arcadeDrive(0, 0.0);
      //new ArmMoveDown(-0.1);
    }
    else{
      m_shintake.setSpeed(0.0);

      m_Drivetrain.drive.arcadeDrive(0, 0);
    }*/

    
    //return new StartEndCommand(()->m_Drivetrain.drive.arcadeDrive(0.2, 0), ()->m_DriveTrain.drive.arcadeDrive(0,0), m_Drivetrain);
    
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
     // m_autonomousCommand.cancel();
    }
    m_Drivetrain.moveDriveTrain(.3, .3);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {


    // Split Arcade Drive
    /*double speed = Robot.m_OI.joy.getRawAxis(2) * 0.5;
    double turn = Robot.m_OI.joy.getRawAxis(1) * 0.85;

    double left = speed + turn;
    double right = speed - turn;

    m_Drivetrain.drive.tankDrive(right, left);*/
    

    // Joystick code
    m_Drivetrain.drive.arcadeDrive(m_OI.j.getZ() * .7, -m_OI.j.getY());
    
    CommandScheduler.getInstance().run();
  }
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
   CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().run();
  }
}