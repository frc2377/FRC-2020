/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import java.util.function.Supplier;

public class DriveSubsystem extends SubsystemBase {

    private final WPI_TalonFX m_leftFrontMasterMotor;
    private final WPI_TalonFX m_rightFrontMasterMotor;

    private final Supplier<Double> leftEncoderPosition;
    private final Supplier<Double> leftEncoderRate;
    private final Supplier<Double> rightEncoderPosition;
    private final Supplier<Double> rightEncoderRate;

    // Define the robot's drive
    private final DifferentialDrive m_drive;

    // Define the gyro sensor
    private final AHRS m_gyro; // NavX
    // private final Gyro m_gyro; // ADXRS450 Analog Gyro

    // Define odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    // Define Kinematics
    private final DifferentialDriveKinematics m_kinematics;

    // Define the drive feedforward
    private final SimpleMotorFeedforward m_feedforward;

    // Define the initial position
    private Pose2d initialPosition;

    // Troubleshooting Section
    NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
    NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");


    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
        // Set up motors for to allow Voltage to be applied. SpeedControllerGroups don't
        // allow for this, so we have to use the Master/Follower setup
        m_leftFrontMasterMotor = new WPI_TalonFX(DriveConstants.kLeftFrontMotorPort);
        m_leftFrontMasterMotor.configFactoryDefault();
        m_leftFrontMasterMotor.setInverted(DriveConstants.kLeftInvertMotor);
        m_leftFrontMasterMotor.setSensorPhase(DriveConstants.kLeftInvertSensorPhase);
        m_leftFrontMasterMotor.setNeutralMode(NeutralMode.Coast);

        m_rightFrontMasterMotor = new WPI_TalonFX(DriveConstants.kRightFrontMotorPort);
        m_rightFrontMasterMotor.configFactoryDefault();
        // Set inverted to true so motors move forward with green LEDs on the right
        m_rightFrontMasterMotor.setInverted(DriveConstants.kRightInvertMotor);
        m_rightFrontMasterMotor.setSensorPhase(DriveConstants.kRightInvertSensorPhase);
        m_rightFrontMasterMotor.setNeutralMode(NeutralMode.Coast);

        WPI_TalonFX leftRearSlaveMotor = new WPI_TalonFX(DriveConstants.kLeftRearMotorPort);
        leftRearSlaveMotor.configFactoryDefault();
        leftRearSlaveMotor.setInverted(InvertType.FollowMaster);
        leftRearSlaveMotor.follow(m_leftFrontMasterMotor);
        leftRearSlaveMotor.setNeutralMode(NeutralMode.Coast);

        WPI_TalonFX rightRearSlaveMotor = new WPI_TalonFX(DriveConstants.kRightRearMotorPort);
        rightRearSlaveMotor.configFactoryDefault();
        // Set inverted to true so motors move forward with green LEDs on the right
        rightRearSlaveMotor.setInverted(InvertType.FollowMaster);
        rightRearSlaveMotor.follow(m_rightFrontMasterMotor);
        rightRearSlaveMotor.setNeutralMode(NeutralMode.Coast);

        // m_gyro = new ADXRS450_Gyro();
        m_gyro = new AHRS(SPI.Port.kMXP);

        /*
         * Configure drivetrain movement
         */
        m_drive = new DifferentialDrive(m_leftFrontMasterMotor, m_rightFrontMasterMotor);
        // Since the right side motors are already inverted, setRightSideInverted is
        // false
        m_drive.setRightSideInverted(false);
        m_drive.setDeadband(0.02); // This is the default

        // Configure the integrated encoders
        m_leftFrontMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.kVelocityPIDSlotIdx0,
                10);
        leftEncoderPosition = () -> m_leftFrontMasterMotor.getSelectedSensorPosition(Constants.kVelocityPIDSlotIdx0)
                * DriveConstants.kDistancePerRevolution;
        leftEncoderRate = () -> m_leftFrontMasterMotor.getSelectedSensorVelocity(Constants.kVelocityPIDSlotIdx0) * 10
                * DriveConstants.kDistancePerRevolution;

        m_rightFrontMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.kVelocityPIDSlotIdx0,
                10);
        rightEncoderPosition = () -> m_rightFrontMasterMotor.getSelectedSensorPosition(Constants.kVelocityPIDSlotIdx0)
                * DriveConstants.kDistancePerRevolution;
        rightEncoderRate = () -> m_rightFrontMasterMotor.getSelectedSensorVelocity(Constants.kVelocityPIDSlotIdx0) * 10
                * DriveConstants.kDistancePerRevolution;
        resetEncoders();

        m_kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackwidthMeters);

        m_odometry = new DifferentialDriveOdometry(getHeading());

        m_feedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter);

        resetEncoders();
        resetGyro();

        // Add sensors to LiveWindow
        addChild("Drive", m_drive);
        addChild("Gyro", m_gyro);
        addChild("Left Master Motor", m_leftFrontMasterMotor);
        addChild("Right Master Motor", m_rightFrontMasterMotor);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_odometry.update(getHeading(), leftEncoderPosition.get(), rightEncoderPosition.get());

        var translation = m_odometry.getPoseMeters().getTranslation();
        m_xEntry.setNumber(translation.getX());
        m_yEntry.setNumber(translation.getY());
        m_drive.feedWatchdog(); //TODO: Remove this
        log();
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometryPose(Pose2d position, Rotation2d rotation) {
        m_odometry.resetPosition(position, rotation);
    }

    /**
     * Returns the drive odometry.
     *
     * @return odometry.
     */
    public DifferentialDriveOdometry getOdometry() {
        return m_odometry;
    }

    /**
     * Returns the drive kinematics.
     *
     * @return kinematics.
     */
    public DifferentialDriveKinematics getKinematics() {
        return m_kinematics;
    }

    /**
     * Returns the drive feedforward.
     *
     * @return feedforward.
     */
    public SimpleMotorFeedforward getFeedforward() {
        return m_feedforward;
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoderRate.get(), rightEncoderRate.get());
    }

    /**
     * Drive the robot using arcade controls
     *
     * @param fwd the commanded forwared movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(final double fwd, final double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }

    /**
     * Control the left and right side of the drive directly with voltage
     *
     * @param leftVolts  the commanded left output in voltage
     * @param rightVolts the commanded right output in voltage
     */
    public void tankDriveVolts(final double leftVolts, final double rightVolts) {
        m_leftFrontMasterMotor.setVoltage(leftVolts);
        m_rightFrontMasterMotor.setVoltage(rightVolts);
        m_drive.feed();
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        m_leftFrontMasterMotor.setSelectedSensorPosition(0);
        m_rightFrontMasterMotor.setSelectedSensorPosition(0);
    }

    /**
     * Resets the heading of the robot
     */
    public void resetGyro() {
        m_gyro.reset();
        m_gyro.zeroYaw();
    }

    /**
     * Tank style driving for the DriveTrain.
     *
     * @param left  Speed in range [-1,1]
     * @param right Speed in range [-1,1]
     */
    public void drive(final double left, final double right) {
        m_drive.tankDrive(left, right);
    }

    public void setMaxOutput(final double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    /**
     * The log method puts interesting information to the SmartDashboard.
     */
    public void log() {
        SmartDashboard.putNumber("Left Speed", m_leftFrontMasterMotor.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Right Speed", m_rightFrontMasterMotor.getSelectedSensorPosition(0));
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0));
    }
}
