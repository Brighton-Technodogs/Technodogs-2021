/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.PulseIntake;
import frc.robot.commands.intake.ReverseIntakeCommand;
import frc.robot.commands.intake.RunIntakeWithSensorCommand;
import frc.robot.commands.intake.RunStorageCommand;
import frc.robot.commands.shooter.AutoShootCommand;
import frc.robot.commands.shooter.AutoShootCommandGroup;
import frc.robot.commands.shooter.LimelightTestingCommand;
import frc.robot.commands.shooter.QuickFireCommand;
import frc.robot.commands.shooter.ReverseShooterCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.SmartDashboardShootingCommand;
import frc.robot.commands.drive.DisableRearLeftRotationCommand;
import frc.robot.commands.drive.DisableRearRightRotationCommand;
import frc.robot.commands.drive.DisableFrontLeftRotationCommand;
import frc.robot.commands.drive.DisableFrontRightRotationCommand;
import frc.robot.commands.drive.AssistedLimelightDriveCommand;
import frc.robot.commands.auto.AutonomousSequentialCommandGroup;

// import frc.robot.commands.drive.EnableRearLeftRotationCommand;
// import frc.robot.commands.drive.EnableRearRightRotationCommand;
// import frc.robot.commands.drive.EnableFrontLeftRotationCommand;
// import frc.robot.commands.drive.EnableFrontRightRotationCommand;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);
  private final AssistedLimelightDriveCommand assistedLimelightDriveCommand = new AssistedLimelightDriveCommand(driveSubsystem);

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ShootCommand shootCommand = new ShootCommand(shooterSubsystem);
  private final LimelightTestingCommand limelightTestingCommand = new LimelightTestingCommand(shooterSubsystem);
  private final AutoShootCommand autoShootCommand = new AutoShootCommand(shooterSubsystem);
  private final SmartDashboardShootingCommand smartDashboardShootingCommand = new SmartDashboardShootingCommand(shooterSubsystem);
  private final QuickFireCommand quickFireCommand = new QuickFireCommand(shooterSubsystem);

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem);
  private final PulseIntake pulseIntake = new PulseIntake(intakeSubsystem);
  private final RunStorageCommand runStorageCommand = new RunStorageCommand(intakeSubsystem);
  private final RunIntakeWithSensorCommand runIntakeWithSensorCommand = new RunIntakeWithSensorCommand(intakeSubsystem);
  private final ReverseShooterCommand reverseShooterCommand = new ReverseShooterCommand(intakeSubsystem);
  private final ReverseIntakeCommand reverseIntakeCommand = new ReverseIntakeCommand(intakeSubsystem);

  private final AutonomousSequentialCommandGroup autonomousSequentialCommandGroup = new AutonomousSequentialCommandGroup(driveSubsystem, shooterSubsystem, intakeSubsystem);

  // private final DisableRearLeftRotationCommand disableRearLeftRotationCommand = new DisableRearLeftRotationCommand(driveSubsystem);
  // private final DisableRearRightRotationCommand disableRearRightRotationCommand = new DisableRearRightRotationCommand(driveSubsystem);
  // private final DisableFrontLeftRotationCommand disableFrontLeftRotationCommand = new DisableFrontLeftRotationCommand(driveSubsystem);
  // private final DisableFrontRightRotationCommand disableFrontRightRotationCommand = new DisableFrontRightRotationCommand(driveSubsystem);

  // private final EnableRearLeftRotationCommand enableRearLeftRotationCommand = new EnableRearLeftRotationCommand(driveSubsystem);
  // private final EnableRearRightRotationCommand enableRearRightRotationCommand = new EnableRearRightRotationCommand(driveSubsystem);
  // private final EnableFrontLeftRotationCommand enableFrontLeftRotationCommand = new EnableFrontLeftRotationCommand(driveSubsystem);
  // private final EnableFrontRightRotationCommand enableFrontRightRotationCommand = new EnableFrontRightRotationCommand(driveSubsystem);

  private final XboxController operatrorController = new XboxController(1);
  private final JoystickButton operatorAButton = new JoystickButton(operatrorController, 1);
  private final JoystickButton operatorBButton = new JoystickButton(operatrorController, 2);
  private final JoystickButton operatorXButton = new JoystickButton(operatrorController, XboxController.Button.kX.value);

  private final XboxController driverController = new XboxController(0);
  private final JoystickButton driverAButton = new JoystickButton(driverController, 1);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    driveSubsystem.setDefaultCommand(assistedLimelightDriveCommand);

    shooterSubsystem.setDefaultCommand(autoShootCommand);

    intakeSubsystem.setDefaultCommand(runIntakeWithSensorCommand);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    System.out.println("Configuring Button Bindings");

    operatorAButton.whenHeld(runStorageCommand);
    operatorBButton.whenHeld(quickFireCommand);
    operatorXButton.whenHeld(reverseIntakeCommand);
    //operatorBButton.whenPressed(limelightTestingCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getDriveCommand() {
    // An ExampleCommand will run in autonomous
    return driveCommand;
  }*/

  public Command getAutoCommand ()
  {
    return autonomousSequentialCommandGroup;
  }
}
