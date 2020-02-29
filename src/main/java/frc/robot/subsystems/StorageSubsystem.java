/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Map;

public class StorageSubsystem extends SubsystemBase {
  
  private VictorSPX innerStorageMotor = new VictorSPX(Constants.IntakeSubsystem.innerStorageVictorCan);

  private ShuffleboardTab subsystemShuffleboardTab = Shuffleboard.getTab("Storage Subsystem");

  public StorageSubsystem() {

  }

  //Run the storage motor at desired speed
  public void runStorage (double speed)
  {
    subsystemShuffleboardTab.add("Inner Storage Motor Output", speed)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", -1, "max", 1)) ;
    innerStorageMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
