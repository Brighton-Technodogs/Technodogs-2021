/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  // ADXRS450_Gyro gyro =  new ADXRS450_Gyro();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    System.out.println("   ______________  _____");
    System.out.println("  |__  /__  / __ \\/__  /");
    System.out.println("   /_ <  / / / / /  / / ");
    System.out.println(" ___/ / / / /_/ /  / /  ");
    System.out.println("/____/ /_/\\____/  /_/   ");
    System.out.println("  ______          __              ");
    System.out.println(" /_  __/__  _____/ /_  ____  ____ ");
    System.out.println("  / / / _ \\/ ___/ __ \\/ __ \\/ __ \\");
    System.out.println(" / / /  __/ /__/ / / / / / / /_/ /");
    System.out.println("/_/ _\\___/\\___/_/ /_/_/ /_/\\____/ ");
    System.out.println("   / __ \\____  ____ ______        ");
    System.out.println("  / / / / __ \\/ __ `/ ___/        ");
    System.out.println(" / /_/ / /_/ / /_/ (__  )         ");
    System.out.println("/_____/\\____/\\__, /____/          ");
    System.out.println("            /____/                ");
    robotContainer = new RobotContainer();
    CameraServer.getInstance().startAutomaticCapture();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() 
  {

    
    

  }

  @Override
  public void disabledPeriodic() 
  {

    //SmartDashboard.putNumber("Area", new ShooterSubsystem().getArea());

    //System.out.println(new ShooterSubsystem().getArea());
    
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    autonomousCommand = robotContainer.getAutoCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  Servo servo;

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    // gyro.calibrate();
    // gyro.reset();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
  {
    // SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
