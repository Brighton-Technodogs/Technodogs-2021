package frc.robot.subsystems;

// import org.usfirst.frc3707.Creedence.Robot;
import frc.robot.Constants;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.SwerveWheel;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {

    private AnalogPotentiometer frontRightEncoder = new AnalogPotentiometer(Constants.DriveSubsystem.kFrontRightEncoderPort, 360.0, 0.0);
    private VictorSP frontRightTwistMotor = new VictorSP(Constants.DriveSubsystem.kFrontRightTwistMotorPort);
    private PIDController frontRightTwistController = new PIDController(0.05, 0.0, 0.0);
    private AnalogPotentiometer frontLeftEncoder = new AnalogPotentiometer(Constants.DriveSubsystem.kFrontLeftEncoderPort, 360.0, 0.0);
    private VictorSP frontLeftTwistMotor = new VictorSP(Constants.DriveSubsystem.kFrontLeftTwistMotorPort);
    private PIDController frontLeftTwistController = new PIDController(0.05, 0.0, 0.0);
    private AnalogPotentiometer backRightEncoder = new AnalogPotentiometer(Constants.DriveSubsystem.kBackRightEncoderPort, 360.0, 0.0);
    private VictorSP backRightTwistMotor = new VictorSP(Constants.DriveSubsystem.kRearRightTwistMotorPort);
    private PIDController backRightTwistController = new PIDController(0.05, 0.0, 0.0);
    private AnalogPotentiometer backLeftEncoder = new AnalogPotentiometer(Constants.DriveSubsystem.kBackLeftEncoderPort, 360.0, 0.0);
    private VictorSP backLeftTwistMotor = new VictorSP(Constants.DriveSubsystem.kRearLeftTwistMotorPort);
    private PIDController backLeftTwistController = new PIDController(0.05, 0.0, 0.0);
    
    private VictorSP frontRightDriveMotor = new VictorSP(Constants.DriveSubsystem.kFrontRightDriveMotorPort);
    private VictorSP frontLeftDriveMotor = new VictorSP(Constants.DriveSubsystem.kFrontLeftDriveMotorPort);
    private VictorSP backRightDriveMotor = new VictorSP(Constants.DriveSubsystem.kRearRightDriveMotorPort);
    private VictorSP backLeftDriveMotor = new VictorSP(Constants.DriveSubsystem.kRearLeftDriveMotorPort);

    // private CANSparkMax frontRightDrive = new CANSparkMax(Constants.DriveSystem.FrontRight.getDrive(), MotorType.kBrushless);
    // private CANSparkMax frontLeftDrive = new CANSparkMax(Constants.DriveSystem.FrontLeft.getDrive(), MotorType.kBrushless);
    // private CANSparkMax backRightDrive = new CANSparkMax(Constants.DriveSystem.BackRight.getDrive(), MotorType.kBrushless);
    // private CANSparkMax backLeftDrive = new CANSparkMax(Constants.DriveSystem.BackLeft.getDrive(), MotorType.kBrushless);


    private SwerveWheel frontRightWheel = new SwerveWheel(frontRightTwistController, frontRightEncoder, frontRightTwistMotor, frontRightDriveMotor, Constants.DriveSubsystem.kFrontRightEncoderOffset);
    private SwerveWheel frontLeftWheel = new SwerveWheel(frontLeftTwistController, frontLeftEncoder, frontLeftTwistMotor, frontLeftDriveMotor,Constants.DriveSubsystem.kFrontLeftEncoderOffset);
    private SwerveWheel backRightWheel = new SwerveWheel(backRightTwistController, backRightEncoder, backRightTwistMotor, backRightDriveMotor, Constants.DriveSubsystem.kRearRightEncoderOffset);
    private SwerveWheel backLeftWheel = new SwerveWheel(backLeftTwistController, backLeftEncoder, backLeftTwistMotor, backLeftDriveMotor, Constants.DriveSubsystem.kRearLeftEncoderOffset);
    public SwerveDrive swerve = new SwerveDrive(frontRightWheel, frontLeftWheel, backLeftWheel, backRightWheel, null);

    public void init() {
        //this is how you set a parameter on the spark... this one sets it to PWM
        //frontLeftDrive.setParameter(com.revrobotics.CANSparkMaxLowLevel.ConfigParameter.kInputMode, CANSparkMax.InputMode.kPWM.value);
        frontRightTwistController.enableContinuousInput(0.0, 360.0);
        frontLeftTwistController.enableContinuousInput(0.0, 360.0);
        backLeftTwistController.enableContinuousInput(0.0, 360.0);
        backRightTwistController.enableContinuousInput(0.0, 360.0);


    }

    /**
     * Enables motors. bcuz it werkz?
     */
    public void enable() {
        frontRightWheel.enableRotation();
        frontLeftWheel.enableRotation();
        backRightWheel.enableRotation();
        backLeftWheel.enableRotation();
    }

    /**
     * Disables motors. bcuz it werkz?
     */
    public void disable() {
        frontRightWheel.disableRotation();
        frontLeftWheel.disableRotation();
        backRightWheel.disableRotation();
        backLeftWheel.disableRotation();
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
        SmartDashboard.putNumber("Front Right Encoder", frontRightEncoder.get());
        SmartDashboard.putNumber("Front Left Encoder", frontLeftEncoder.get());
        SmartDashboard.putNumber("Back Right Encoder", backRightEncoder.get());
        SmartDashboard.putNumber("Back Left Encoder", backLeftEncoder.get());
    }

    public void disableFrontRightWheelRotation(){
        frontRightWheel.disableRotation();
    }
    public void disableFrontLeftWheelRotation(){
        frontLeftWheel.disableRotation();
    }
    public void disableBackRightWheelRotation(){
        backRightWheel.disableRotation();
    }
    public void disableBackLeftWheelRotation(){
        backLeftWheel.disableRotation();
    }
    public void enableFrontRightWheelRotation(){
        frontRightWheel.enableRotation();
    }
    public void enableFrontLeftWheelRotation(){
        frontLeftWheel.enableRotation();
    }
    public void enableBackRightWheelRotation(){
        backRightWheel.enableRotation();
    }
    public void enableBackLeftWheelRotation(){
        backLeftWheel.enableRotation();
    }
}
