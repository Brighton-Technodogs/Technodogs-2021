package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import org.usfirst.frc3707.Creedence.Robot;
import frc.robot.Constants;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.SwerveWheel;

public class DriveSubsystem extends SubsystemBase {

    //Swerve Modules

    private AnalogPotentiometer frontRightEncoder = new AnalogPotentiometer(Constants.DriveSubsystem.kFrontRightEncoderPort, 360.0, 0.0);
    private VictorSPX frontRightTwistMotor = new VictorSPX(Constants.DriveSubsystem.kFrontRightTwistMotorCanID);
    private PIDController frontRightTwistController = new PIDController(Constants.DriveSubsystem.kSwerveTwistPID_P, 0.0, 0.0);
    private TalonFX frontRightDriveMotor = new TalonFX(Constants.DriveSubsystem.kFrontRightDriveMotorCanID);
    private SwerveWheel frontRightWheel = new SwerveWheel(frontRightTwistController, frontRightEncoder, frontRightTwistMotor, frontRightDriveMotor, Constants.DriveSubsystem.kFrontRightEncoderOffset, "FrontRight");

    private AnalogPotentiometer frontLeftEncoder = new AnalogPotentiometer(Constants.DriveSubsystem.kFrontLeftEncoderPort, 360.0, 0.0);
    private VictorSPX frontLeftTwistMotor = new VictorSPX(Constants.DriveSubsystem.kFrontLeftTwistMotorCanID);
    private PIDController frontLeftTwistController = new PIDController(Constants.DriveSubsystem.kSwerveTwistPID_P, 0.0, 0.0);
    private TalonFX frontLeftDriveMotor = new TalonFX(Constants.DriveSubsystem.kFrontLeftDriveMotorCanID);
    private SwerveWheel frontLeftWheel = new SwerveWheel(frontLeftTwistController, frontLeftEncoder, frontLeftTwistMotor, frontLeftDriveMotor,Constants.DriveSubsystem.kFrontLeftEncoderOffset, "FrontLeft");

    private AnalogPotentiometer rearRightEncoder = new AnalogPotentiometer(Constants.DriveSubsystem.kRearRightEncoderPort, 360.0, 0.0);
    private VictorSPX rearRightTwistMotor = new VictorSPX(Constants.DriveSubsystem.kRearRightTwistMotorCanID);
    private PIDController rearRightTwistController = new PIDController(Constants.DriveSubsystem.kSwerveTwistPID_P, 0.0, 0.0);
    private TalonFX rearRightDriveMotor = new TalonFX(Constants.DriveSubsystem.kRearRightDriveMotorCanID);
    private SwerveWheel rearRightWheel = new SwerveWheel(rearRightTwistController, rearRightEncoder, rearRightTwistMotor, rearRightDriveMotor, Constants.DriveSubsystem.kRearRightEncoderOffset, "RearRight");

    private AnalogPotentiometer rearLeftEncoder = new AnalogPotentiometer(Constants.DriveSubsystem.kRearLeftEncoderPort, 360.0, 0.0);
    private VictorSPX rearLeftTwistMotor = new VictorSPX(Constants.DriveSubsystem.kRearLeftTwistMotorCanID);
    private PIDController rearLeftTwistController = new PIDController(Constants.DriveSubsystem.kSwerveTwistPID_P, 0.0, 0.0);
    private TalonFX rearLeftDriveMotor = new TalonFX(Constants.DriveSubsystem.kRearLeftDriveMotorCanID);
    private SwerveWheel rearLeftWheel = new SwerveWheel(rearLeftTwistController, rearLeftEncoder, rearLeftTwistMotor, rearLeftDriveMotor, Constants.DriveSubsystem.kRearLeftEncoderOffset, "RearLeft");
    
    //end swerve modules

    //swerve object using all swerve parts
    public SwerveDrive swerve = new SwerveDrive(frontRightWheel, frontLeftWheel, rearLeftWheel, rearRightWheel, null);

    private ShuffleboardTab subsystemShuffleboardTab = Shuffleboard.getTab("Drive Subsystem");

    // Shuffleboard 
    private NetworkTableEntry sbFowardInput = subsystemShuffleboardTab.add("Forward Input", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sbSidewaysInput = subsystemShuffleboardTab.add("Sideways Input", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sbrotation = subsystemShuffleboardTab.add("rotation", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sFieldRelative = subsystemShuffleboardTab.add("fieldOriented", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();

    private NetworkTableEntry sbFLEncoderRaw = subsystemShuffleboardTab.add("FLEncoder Raw", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sbFREncoderRaw = subsystemShuffleboardTab.add("FREncoder Raw", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sbRLEncoderRaw = subsystemShuffleboardTab.add("RLEncoder Raw", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sbRREncoderRaw = subsystemShuffleboardTab.add("RREncoder Raw", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    public void init() {
        System.out.println("Initializing DriveSubsystem");
        setupEncoders();
    }

    private void setupEncoders() {

        //set twist motors to continuous 360 movement
        frontRightTwistController.enableContinuousInput(0.0, 360.0);
        frontLeftTwistController.enableContinuousInput(0.0, 360.0);
        rearLeftTwistController.enableContinuousInput(0.0, 360.0);
        rearRightTwistController.enableContinuousInput(0.0, 360.0);

        // frontRightTwistController.disableContinuousInput();
        // frontLeftTwistController.disableContinuousInput();
        // rearLeftTwistController.disableContinuousInput();
        // rearRightTwistController.disableContinuousInput();

        frontRightTwistController.setTolerance(Constants.DriveSubsystem.kSwerveTwistPIDTolerance);
        frontLeftTwistController.setTolerance(Constants.DriveSubsystem.kSwerveTwistPIDTolerance);
        rearLeftTwistController.setTolerance(Constants.DriveSubsystem.kSwerveTwistPIDTolerance);
        rearRightTwistController.setTolerance(Constants.DriveSubsystem.kSwerveTwistPIDTolerance);

    }

    public void enable() {
        System.out.println("Enabling DriveSubsystem");
        frontRightWheel.enableRotation();
        frontLeftWheel.enableRotation();
        rearRightWheel.enableRotation();
        rearLeftWheel.enableRotation();
    }

    /*public void disable() {
        System.out.println("Disabling DriveSubsystem");
        frontRightWheel.disableRotation();
        frontLeftWheel.disableRotation();
        rearRightWheel.disableRotation();
        rearLeftWheel.disableRotation();
    }*/

    /**
     * Drives the robot based on parameter values
     * 
     * @param directionX Proportional speed at which to move left to right
     * @param directionY Proportional speed at which to move front to back
     * @param rotation   Proportional speed at which to rotate
     * @param useGyro    Boolean for field-oriented driving
     * @param slowSpeed  Boolean for slow mode to make the robot drive slower.
     * @param noPush     Boolean to lock wheels at 45 degree angles, to prevent the
     *                   robot from being pushed in any direction
     */
    public void drive(double directionX, double directionY, double rotation, boolean useGyro, boolean slowSpeed,
            boolean noPush) {
        swerve.drive(directionX * 0.5, directionY * 0.5, rotation * 0.34, false, slowSpeed, noPush);
    }

    //rotates the swerves to a circle and runs the motors at a desired speed
    public void CircleDrive(double speed)
    {
        swerve.CircleRotate(speed);
    }

    public void driveSimple(double speed, double angle)
    {
        swerve.driveSimple(speed, angle);
    }

    public void xMode()
    {
        swerve.XModeActivate();
    }

    /**
     * The function which executes periodically to run the DriveTrain subsystem
     */
    @Override
    public void periodic() {
        publishDataToSmartDashboard();
    }

    private void publishDataToSmartDashboard() {

      // sbFLEncoderRaw.setDouble(m_frontLeft.getRawAngle());
      // sbFREncoderRaw.setDouble(m_frontRight.getRawAngle());
      // sbRLEncoderRaw.setDouble(m_rearLeft.getRawAngle());
      // sbRREncoderRaw.setDouble(m_rearRight.getRawAngle());

        // Publish encoder values to smart dashboard for offset tuning
        SmartDashboard.putNumber("Front Right Encoder", frontRightEncoder.get());
        SmartDashboard.putNumber("Front Left Encoder", frontLeftEncoder.get());
        SmartDashboard.putNumber("Back Right Encoder", rearRightEncoder.get());
        SmartDashboard.putNumber("Back Left Encoder", rearLeftEncoder.get());

    }

    
    public void disableFrontRightWheelRotation(){
        frontRightWheel.disableRotation();
    }
    public void disableFrontLeftWheelRotation(){
        frontLeftWheel.disableRotation();
    }
    public void disableRearRightWheelRotation(){
        rearRightWheel.disableRotation();
    }
    public void disableRearLeftWheelRotation(){
        rearLeftWheel.disableRotation();
    }
    public void enableFrontRightWheelRotation(){
        frontRightWheel.enableRotation();
    }
    public void enableFrontLeftWheelRotation(){
        frontLeftWheel.enableRotation();
    }
    public void enableRearRightWheelRotation(){
        rearRightWheel.enableRotation();
    }
    public void enableRearLeftWheelRotation(){
        rearLeftWheel.enableRotation();
    }
}