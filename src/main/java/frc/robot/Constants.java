/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;

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
        public static final int driverControllerPort = 0;
        public static final int driverControllerLeftStickXAxis = 0;
        public static final int driverControllerLeftStickYAxis = 1;
        public static final int driverControllerRightStickXAxis = 4;
        public static final int driverControllerRightStickYAxis = 5;
        public static final int driverControllerRightTriggerAxis = 3;
    }

    public final class OperatorControl {
        public static final int operatorControllerPort = 1;
        public static final int operatorRightTrigger = 3;
        public static final int operatorLeftTrigger = 2;
    }

    public static final class DriveSubsystem {

        public static final double kSwerveTwistPIDTolerance = 10;
        public static final double kSwerveTwistPID_P = 0.05;
        public static final double kSwerveTwistPID_I = 0;
        public static final double kSwerveTwistPID_D = 0;

        public static final double kSwerveDrivePIDTolerance = 10;
        public static final double kSwerveDrivePID_P = 0;
        public static final double kSwerveDrivePID_I = 0;
        public static final double kSwerveDrivePID_D = 0;
        public static final double kSwerveDrivePID_F = 0.052;

        public static final double kMaxTwistAngularVelocity = 360; // deg/s
        public static final double kMaxTwistAngularAcceleration = 360; // deg/s^2

        public static final double kGearRatioMotorToWheel = 6.64; // 6.64 motor rotations = 1 wheel rotation
        public static final double kWheelDiameter = 0.1524; // in meters
        public static final double kMotorEncoderTicksPerRev = 2048;
        public static final double kMotorEncoderTimeUnit = 0.100; // 100ms

        //kPXController

        public static final boolean kGyroReversed = true;

        public static final double kWheelBase = 0.5842;  // in meters. = 23 inches
        public static final double kTrackWidth = 0.5588; // in meters. = 22 inches

        public static final double kMaxSpeedMetersPerSecond = 7;

        public static final double kMaxAccelerationMetersPerSecondSquared = 1; // yeet

        public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
          new Translation2d(Constants.DriveSubsystem.kWheelBase / 2, Constants.DriveSubsystem.kTrackWidth / 2),
          new Translation2d(Constants.DriveSubsystem.kWheelBase / 2, -Constants.DriveSubsystem.kTrackWidth / 2),
          new Translation2d(-Constants.DriveSubsystem.kWheelBase / 2, Constants.DriveSubsystem.kTrackWidth / 2),
          new Translation2d(-Constants.DriveSubsystem.kWheelBase / 2, -Constants.DriveSubsystem.kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorCanID = 2;
        public static final int kRearLeftDriveMotorCanID = 1;
        public static final int kFrontRightDriveMotorCanID = 3;
        public static final int kRearRightDriveMotorCanID = 4;

        public static final int kFrontLeftTwistMotorCanID = 12;
        public static final int kRearLeftTwistMotorCanID = 11;
        public static final int kFrontRightTwistMotorCanID = 13;
        public static final int kRearRightTwistMotorCanID = 14;

        public static final int kFrontLeftEncoderPort = 1;
        public static final int kRearLeftEncoderPort = 0;
        public static final int kFrontRightEncoderPort = 2;
        public static final int kRearRightEncoderPort = 3;

        public static final double kFrontLeftEncoderOffset = 160; //352; //60
        public static final double kFrontRightEncoderOffset = 175;// 4; //28
        public static final double kRearLeftEncoderOffset = 14; // 191; //11
        public static final double kRearRightEncoderOffset = 76; //251; //18
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
        public static final int storageContrastSensorDIO = 0;
    }
}