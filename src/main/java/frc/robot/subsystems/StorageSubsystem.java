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
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class StorageSubsystem extends SubsystemBase {
  
  private VictorSPX innerStorageMotor = new VictorSPX(Constants.IntakeSubsystem.innerStorageVictorCan);

  ShuffleboardTab storageTab = Shuffleboard.getTab("Storage Subsystem Tab");

  public StorageSubsystem() 
  {
    
  }

  //Run the storage motor at desired speed
  public void runStorage (double speed)
  {
    // storageSpeed.setDouble(speed);
    innerStorageMotor.set(ControlMode.PercentOutput, speed);

    //storageTab.addPersistent("Inner Storage Motor", speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
