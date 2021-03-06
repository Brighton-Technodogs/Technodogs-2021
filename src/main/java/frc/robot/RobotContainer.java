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

import frc.robot.commands.shooter.AutoShootCommand;
import frc.robot.commands.shooter.ChangeConfigCommand;
import frc.robot.commands.shooter.LongShootCommand;
import frc.robot.commands.shooter.QuickFireBackSpinCommand;
import frc.robot.commands.shooter.QuickFireCommand;
import frc.robot.commands.storage.ReverseStorageCommand;
import frc.robot.commands.storage.RunStorageCommand;
import frc.robot.commands.storage.RunStorageWithSensorCommand;
import frc.robot.commands.drive.AssistedLimelightDriveCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.ResetIntakeCommand;
import frc.robot.commands.intake.ReverseIntakeCommand;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.auto.AutonomousSequentialCommandGroup;
import frc.robot.commands.auto.AutoShootWithXModeSequesntialCommand;
import frc.robot.commands.auto.AutonomousBackwardsShootingSequentialCommand;
import frc.robot.commands.climb.RunClimbCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //Subsystems
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final StorageSubsystem storageSubsystem = new StorageSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  //Drive Commands
  private final AssistedLimelightDriveCommand assistedLimelightDriveCommand = new AssistedLimelightDriveCommand(driveSubsystem);

  //Shooter Commands
  private final AutoShootCommand autoShootCommand = new AutoShootCommand(shooterSubsystem);
  private final QuickFireCommand quickFireCommand = new QuickFireCommand(shooterSubsystem);
  private final QuickFireBackSpinCommand quickFireBackSpinCommand = new QuickFireBackSpinCommand(shooterSubsystem);
  private final ChangeConfigCommand changeConfigCommand = new ChangeConfigCommand(shooterSubsystem);
  private final LongShootCommand longShootCommand = new LongShootCommand(shooterSubsystem);

  //Storage Commands
  private final RunStorageCommand runStorageCommand = new RunStorageCommand(storageSubsystem);
  private final ReverseStorageCommand reverseStorageCommand = new ReverseStorageCommand(storageSubsystem);
  private final RunStorageWithSensorCommand runStorageWithSensorCommand = new RunStorageWithSensorCommand(storageSubsystem);

  //Intake Commands
  private final RunIntakeCommand runIntakeCommand = new RunIntakeCommand(intakeSubsystem);
  private final ReverseIntakeCommand reverseIntakeCommand = new ReverseIntakeCommand(intakeSubsystem);
  private final DeployIntakeCommand deployIntakeCommand = new DeployIntakeCommand(intakeSubsystem);
  private final ResetIntakeCommand resetIntakeCommand = new ResetIntakeCommand(intakeSubsystem);

  //Climb Commands
  private final RunClimbCommand runClimbCommand = new RunClimbCommand(climbSubsystem);
  
  //Auto Commands
  private final AutonomousSequentialCommandGroup autonomousSequentialCommandGroup = new AutonomousSequentialCommandGroup(driveSubsystem, shooterSubsystem, storageSubsystem, intakeSubsystem);
  private final AutonomousBackwardsShootingSequentialCommand autonomousBackwardsShootingSequentialCommand = new AutonomousBackwardsShootingSequentialCommand(driveSubsystem, shooterSubsystem, storageSubsystem, intakeSubsystem);
  private final AutoShootWithXModeSequesntialCommand autoShootWithXModeSequesntialCommand = new AutoShootWithXModeSequesntialCommand(driveSubsystem, shooterSubsystem, storageSubsystem, intakeSubsystem);

  //Operator Contoller and Buttons
  private final XboxController operatorController = new XboxController(1);
  private final JoystickButton operatorBButton = new JoystickButton(operatorController, XboxController.Button.kB.value);
  private final JoystickButton operatorAButton = new JoystickButton(operatorController, XboxController.Button.kA.value);
  private final JoystickButton operatorXButton = new JoystickButton(operatorController, XboxController.Button.kX.value);
  private final JoystickButton operatorYButton = new JoystickButton(operatorController, XboxController.Button.kY.value);
  private final JoystickButton operatorLeftBumper = new JoystickButton(operatorController, XboxController.Button.kBumperLeft.value);
  private final JoystickButton operatorStartButton = new JoystickButton(operatorController, XboxController.Button.kStart.value);
  private final JoystickButton operatorSelectButton = new JoystickButton(operatorController, XboxController.Button.kBack.value);
  private final JoystickButton operatorRightBumper = new JoystickButton(operatorController, XboxController.Button.kBumperRight.value);

    //Driver Controller and Buttons
    private final XboxController driverController = new XboxController(0);
    private final JoystickButton driverStartButton = new JoystickButton(driverController, XboxController.Button.kStart.value);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveSubsystem.setDefaultCommand(assistedLimelightDriveCommand);

    shooterSubsystem.setDefaultCommand(autoShootCommand);

    intakeSubsystem.setDefaultCommand(runIntakeCommand);

    climbSubsystem.setDefaultCommand(runClimbCommand);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    System.out.println("Configuring Button Bindings");

    operatorBButton.whenHeld(quickFireCommand);
    operatorYButton.whenHeld(quickFireBackSpinCommand);

    operatorRightBumper.whenHeld(longShootCommand);

    operatorAButton.whenHeld(runStorageCommand);

    operatorXButton.whenHeld(reverseStorageCommand);

    operatorLeftBumper.whenHeld(reverseIntakeCommand);

    operatorStartButton.whenPressed(deployIntakeCommand);

    operatorSelectButton.whenPressed(resetIntakeCommand);
  }


  //get the auto command
  public Command getAutoCommand ()
  {
    //This auton rotates, shoots, moves forward
    // return autonomousSequentialCommandGroup;

    //this auton moves backwards, intakes, rotates, then shoots
    // return autonomousBackwardsShootingSequentialCommand;

    //This auton rotates, xmodes, shoots, moves forawrd
    return autoShootWithXModeSequesntialCommand;
    
  }
}
