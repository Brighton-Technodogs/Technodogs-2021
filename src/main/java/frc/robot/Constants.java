/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class DriverControl {
        public static final int kDriverControllerPort = 0;
        public static final int kDriverControllerLeftStickXAxis = 0;
        public static final int kDriverControllerLeftStickYAxis = 1;
        public static final int kDriverControllerRightStickXAxis = 4;
        public static final int kDriverControllerRightStickYAxis = 5;
    }

    public final class XboxAxixMapping {
        public static final int operatorControllerPort = 1;
        public static final int operatorRightTrigger = 3;
        public static final int operatorLeftTrigger = 2;
    }

    public final class DriveSubsystem {
        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kRearLeftDriveMotorPort = 3;
        public static final int kFrontRightDriveMotorPort = 1;
        public static final int kRearRightDriveMotorPort = 0;

        public static final int kFrontLeftDriveMotorCanID = 4;
        public static final int kRearLeftDriveMotorCanID = 3;
        public static final int kFrontRightDriveMotorCanID = 1;
        public static final int kRearRightDriveMotorCanID = 2;

        public static final int kFrontLeftTwistMotorPort = 14;
        public static final int kRearLeftTwistMotorPort = 13;
        public static final int kFrontRightTwistMotorPort = 11;
        public static final int kRearRightTwistMotorPort = 12;

        public static final int kFrontLeftEncoderPort = 3;
        public static final int kBackLeftEncoderPort = 2;
        public static final int kFrontRightEncoderPort = 0;
        public static final int kBackRightEncoderPort = 1;

        public static final double kFrontLeftEncoderOffset = 210; //60
        public static final double kRearLeftEncoderOffset = 148; //11
        public static final double kFrontRightEncoderOffset = 154; //28
        public static final double kRearRightEncoderOffset = 256; //18
    }

    public final class ShooterSubsystem
    {
        public static final int bottomShooterFalconCan = 21;
        public static final int leftShooterFalconCan = 22;
        public static final int rightShooterFalconCan = 23;
    }

    public final class IntakeSubsystem
    {
        public static final int innerStorageVictorCan = 31;

        public static final int outerIntakeVictorCan = 32;
    }

    public final class Sensors
    {
        public static final int intakeContrastSensorDIO = 0;
    }
}