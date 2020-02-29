package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import org.usfirst.frc3707.Creedence.Robot;
import frc.robot.Constants;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.SwerveWheel;


public class DriveSubsystem extends SubsystemBase {

    //Swerve Modules

    private final AnalogPotentiometer frontRightEncoder = new AnalogPotentiometer(
            Constants.DriveSubsystem.kFrontRightEncoderPort, 360.0, 0.0);
    private final VictorSPX frontRightTwistMotor = new VictorSPX(Constants.DriveSubsystem.kFrontRightTwistMotorPort);
    private final PIDController frontRightTwistController = new PIDController(
            Constants.DriveSubsystem.kSwerveTwistPID_P, 0.0, 0.0);
    private final TalonFX frontRightDriveMotor = new TalonFX(Constants.DriveSubsystem.kFrontRightDriveMotorCanID);
    private final SwerveWheel frontRightWheel = new SwerveWheel(frontRightTwistController, frontRightEncoder,
            frontRightTwistMotor, frontRightDriveMotor, Constants.DriveSubsystem.kFrontRightEncoderOffset,
            "FrontRight");

    private final AnalogPotentiometer frontLeftEncoder = new AnalogPotentiometer(
            Constants.DriveSubsystem.kFrontLeftEncoderPort, 360.0, 0.0);
    private final VictorSPX frontLeftTwistMotor = new VictorSPX(Constants.DriveSubsystem.kFrontLeftTwistMotorPort);
    private final PIDController frontLeftTwistController = new PIDController(Constants.DriveSubsystem.kSwerveTwistPID_P,
            0.0, 0.0);
    private final TalonFX frontLeftDriveMotor = new TalonFX(Constants.DriveSubsystem.kFrontLeftDriveMotorCanID);
    private final SwerveWheel frontLeftWheel = new SwerveWheel(frontLeftTwistController, frontLeftEncoder,
            frontLeftTwistMotor, frontLeftDriveMotor, Constants.DriveSubsystem.kFrontLeftEncoderOffset, "FrontLeft");

    private final AnalogPotentiometer rearRightEncoder = new AnalogPotentiometer(
            Constants.DriveSubsystem.kBackRightEncoderPort, 360.0, 0.0);
    private final VictorSPX rearRightTwistMotor = new VictorSPX(Constants.DriveSubsystem.kRearRightTwistMotorPort);
    private final PIDController rearRightTwistController = new PIDController(Constants.DriveSubsystem.kSwerveTwistPID_P,
            0.0, 0.0);
    private final TalonFX rearRightDriveMotor = new TalonFX(Constants.DriveSubsystem.kRearRightDriveMotorCanID);
    private final SwerveWheel rearRightWheel = new SwerveWheel(rearRightTwistController, rearRightEncoder,
            rearRightTwistMotor, rearRightDriveMotor, Constants.DriveSubsystem.kRearRightEncoderOffset, "RearRight");

    private final AnalogPotentiometer rearLeftEncoder = new AnalogPotentiometer(
            Constants.DriveSubsystem.kBackLeftEncoderPort, 360.0, 0.0);
    private final VictorSPX rearLeftTwistMotor = new VictorSPX(Constants.DriveSubsystem.kRearLeftTwistMotorPort);
    private final PIDController rearLeftTwistController = new PIDController(Constants.DriveSubsystem.kSwerveTwistPID_P,
            0.0, 0.0);
    private final TalonFX rearLeftDriveMotor = new TalonFX(Constants.DriveSubsystem.kRearLeftDriveMotorCanID);
    private final SwerveWheel rearLeftWheel = new SwerveWheel(rearLeftTwistController, rearLeftEncoder,
            rearLeftTwistMotor, rearLeftDriveMotor, Constants.DriveSubsystem.kRearLeftEncoderOffset, "RearLeft");

    // end swerve modules

    // swerve object using all swerve parts
    public SwerveDrive swerve = new SwerveDrive(frontRightWheel, frontLeftWheel, rearLeftWheel, rearRightWheel, null);
    private ShuffleboardTab subsystemShuffleboardTab = Shuffleboard.getTab("Drive Subsystem");

    // Shuffleboard 
    private NetworkTableEntry sbdirectionX = subsystemShuffleboardTab.add("direction X", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
    private NetworkTableEntry sbdirectionY = subsystemShuffleboardTab.add("direction Y", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
    private NetworkTableEntry sbrotation = subsystemShuffleboardTab.add("rotation", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
    private NetworkTableEntry sbuseGyro = subsystemShuffleboardTab.add("useGyro", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
    private NetworkTableEntry sbslowSpeed = subsystemShuffleboardTab.add("slowSpeed", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
    private NetworkTableEntry sbnoPush = subsystemShuffleboardTab.add("noPush", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
    private NetworkTableEntry sbencoderFR = subsystemShuffleboardTab.add("Encoder Front Right", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
    private NetworkTableEntry sbencoderFL = subsystemShuffleboardTab.add("Encoder Front Left", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
    private NetworkTableEntry sbencoderBR = subsystemShuffleboardTab.add("Encoder Back Right", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
    private NetworkTableEntry sbencoderBL = subsystemShuffleboardTab.add("Encoder Back Left", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    // Shuffeboard Graphed
    private NetworkTableEntry gsbdirectionX = subsystemShuffleboardTab.add("graphed direction X", 0)
    .withWidget(BuiltInWidgets.kGraph)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
    private NetworkTableEntry gsbdirectionY = subsystemShuffleboardTab.add("graphed direction Y", 0)
    .withWidget(BuiltInWidgets.kGraph)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
    private NetworkTableEntry gsbrotation = subsystemShuffleboardTab.add("graphed rotation", 0)
    .withWidget(BuiltInWidgets.kGraph)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();



    public void init() {
        System.out.println("Initializing DriveSubsystem");
        setupEncoders();
    }

    private void setupEncoders() {

        // set twist motors to continuous 360 movement
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
    public void drive(final double directionX, final double directionY, final double rotation, final boolean useGyro,
            final boolean slowSpeed, final boolean noPush) {

        sbdirectionX.setDouble(directionX);
        sbdirectionY.setDouble(directionX);
        sbrotation.setDouble(rotation);
        gsbdirectionX.setDouble(directionX);
        gsbdirectionY.setDouble(directionX);
        gsbrotation.setDouble(rotation);
        sbuseGyro.setBoolean(useGyro);
        sbslowSpeed.setBoolean(slowSpeed);
        sbnoPush.setBoolean(noPush);
        swerve.drive(directionX, directionY, rotation, false, slowSpeed, noPush);
    }

    // rotates the swerves to a circle and runs the motors at a desired speed
    public void CircleDrive(final double speed)
    {
        swerve.CircleRotate(speed);
    }

    /**
     * The function which executes periodically to run the DriveTrain subsystem
     */
    @Override
    public void periodic() {
        publishDataToShuffleBoard();    
    }

    private void publishDataToShuffleBoard() {
        sbencoderFR.setNumber(frontRightEncoder.get());
        sbencoderFL.setNumber(frontLeftEncoder.get());
        sbencoderBR.setNumber(rearRightEncoder.get());
        sbencoderBL.setNumber(rearLeftEncoder.get());
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
