package frc.robot.subsystems;

// import org.usfirst.frc3707.Creedence.Robot;
import frc.robot.Constants;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.SwerveWheel;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.fasterxml.jackson.annotation.JsonPropertyDescription;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {

    private AnalogPotentiometer frontRightEncoder = new AnalogPotentiometer(Constants.DriveSubsystem.kFrontRightEncoderPort, 360.0, 0.0);
    private VictorSP frontRightTwistMotor = new VictorSP(Constants.DriveSubsystem.kFrontRightTwistMotorPort);
    private PIDController frontRightTwistController = new PIDController(0.05, 0.0, 0.0);
    private TalonFX frontRightDriveMotor = new TalonFX(Constants.DriveSubsystem.kFrontRightDriveMotorCanID);
    private SwerveWheel frontRightWheel = new SwerveWheel(frontRightTwistController, frontRightEncoder, frontRightTwistMotor, frontRightDriveMotor, Constants.DriveSubsystem.kFrontRightEncoderOffset);

    private AnalogPotentiometer frontLeftEncoder = new AnalogPotentiometer(Constants.DriveSubsystem.kFrontLeftEncoderPort, 360.0, 0.0);
    private VictorSP frontLeftTwistMotor = new VictorSP(Constants.DriveSubsystem.kFrontLeftTwistMotorPort);
    private PIDController frontLeftTwistController = new PIDController(0.05, 0.0, 0.0);
    private TalonFX frontLeftDriveMotor = new TalonFX(Constants.DriveSubsystem.kFrontLeftDriveMotorCanID);
    private SwerveWheel frontLeftWheel = new SwerveWheel(frontLeftTwistController, frontLeftEncoder, frontLeftTwistMotor, frontLeftDriveMotor,Constants.DriveSubsystem.kFrontLeftEncoderOffset);

    private AnalogPotentiometer rearRightEncoder = new AnalogPotentiometer(Constants.DriveSubsystem.kBackRightEncoderPort, 360.0, 0.0);
    private VictorSP rearRightTwistMotor = new VictorSP(Constants.DriveSubsystem.kRearRightTwistMotorPort);
    private PIDController rearRightTwistController = new PIDController(0.05, 0.0, 0.0);
    private TalonFX rearRightDriveMotor = new TalonFX(Constants.DriveSubsystem.kRearRightDriveMotorCanID);
    private SwerveWheel rearRightWheel = new SwerveWheel(rearRightTwistController, rearRightEncoder, rearRightTwistMotor, rearRightDriveMotor, Constants.DriveSubsystem.kRearRightEncoderOffset);


    private AnalogPotentiometer rearLeftEncoder = new AnalogPotentiometer(Constants.DriveSubsystem.kBackLeftEncoderPort, 360.0, 0.0);
    private VictorSP rearLeftTwistMotor = new VictorSP(Constants.DriveSubsystem.kRearLeftTwistMotorPort);
    private PIDController rearLeftTwistController = new PIDController(0.05, 0.0, 0.0);
    private TalonFX rearLeftDriveMotor = new TalonFX(Constants.DriveSubsystem.kRearLeftDriveMotorCanID);
    private SwerveWheel rearLeftWheel = new SwerveWheel(rearLeftTwistController, rearLeftEncoder, rearLeftTwistMotor, rearLeftDriveMotor, Constants.DriveSubsystem.kRearLeftEncoderOffset);
    
    public SwerveDrive swerve = new SwerveDrive(frontRightWheel, frontLeftWheel, rearLeftWheel, rearRightWheel, null);


    public void init() {
        System.out.println("Initializing DriveSubsystem");
        setupEncoders();

    }

    private void setupEncoders() {
        frontRightTwistController.enableContinuousInput(0.0, 360.0);
        frontLeftTwistController.enableContinuousInput(0.0, 360.0);
        rearLeftTwistController.enableContinuousInput(0.0, 360.0);
        rearRightTwistController.enableContinuousInput(0.0, 360.0);

    }

    /**
     * Enables motors. bcuz it werkz?
     */
    public void enable() {
        System.out.println("Enabling DriveSubsystem");
        frontRightWheel.enableRotation();
        frontLeftWheel.enableRotation();
        rearRightWheel.enableRotation();
        rearLeftWheel.enableRotation();
    }

    /**
     * Disables motors. bcuz it werkz?
     */
    public void disable() {
        System.out.println("Disabling DriveSubsystem");
        frontRightWheel.disableRotation();
        frontLeftWheel.disableRotation();
        rearRightWheel.disableRotation();
        rearLeftWheel.disableRotation();
    }

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
        swerve.drive(directionX, directionY, rotation, false, slowSpeed, noPush);
    }

    /**
     * The function which executes periodically to run the DriveTrain subsystem
     */
    @Override
    public void periodic() {
        publishDataToSmartDashboard();
    }

    private void publishDataToSmartDashboard() {

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