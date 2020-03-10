/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import java.util.Map;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;



import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;




public class IntakeSubsystem extends SubsystemBase {
  
  private VictorSPX intakeMotor = new VictorSPX(Constants.IntakeSubsystem.outerIntakeVictorCan);
  private ShuffleboardTab subsystemShuffleboardTab = Shuffleboard.getTab("Intake Subsystem");
  private NetworkTableEntry intakeSpeed = 
        subsystemShuffleboardTab.add("Intake %OP speed", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", -1, "max", 1)) 
            .getEntry();


  private Servo deployingServo = new Servo(9);

  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {

  }
  
  //run the intake motor at desired speed
  public void runIntake (double speed)
  {
    intakeSpeed.setDouble(-speed);
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void deployIntake ()
  {
    deployingServo.set(0);
  }

  public void resetIntake()
  {
    deployingServo.set(0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public static void setDefaultCommand(String intakecommand) {
}
}
