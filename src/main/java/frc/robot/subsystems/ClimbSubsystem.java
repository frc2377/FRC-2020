/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// Coded by Logan Cuttler

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {

    private final WPI_TalonSRX m_frontClimbMotor;
    private final WPI_TalonSRX m_rearClimbMotor;
    private final Solenoid m_climberSolenoid;
    private final AHRS m_gyro;

    /**
     * Creates a new ClimbSubsystem.
     */
    public ClimbSubsystem() {
        // Configure front motor
        m_frontClimbMotor = new WPI_TalonSRX(ClimbConstants.kFrontClimbMotorPort);
        m_frontClimbMotor.configFactoryDefault();
        // Set front motor limits
//        m_frontClimbMotor.configForwardSoftLimitThreshold(ClimbConstants.kFrontForwardSoftLimit, 0);
        m_frontClimbMotor.configReverseSoftLimitThreshold(ClimbConstants.kFrontReverseSoftLimit, 0);
//        m_frontClimbMotor.configForwardSoftLimitEnable(true, 0);
        m_frontClimbMotor.configReverseSoftLimitEnable(true, 0);
        m_frontClimbMotor.setNeutralMode(NeutralMode.Brake);
        m_frontClimbMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kVelocityPIDSlotIdx0, Constants.kTimeoutMs);
        m_frontClimbMotor.setInverted(ClimbConstants.kFrontInvertMotor);
        m_frontClimbMotor.setSensorPhase(ClimbConstants.kFrontInvertSensorDirection);
        m_frontClimbMotor.config_kF(Constants.kVelocityPIDSlotIdx0, ClimbConstants.kF_front, Constants.kTimeoutMs);
        m_frontClimbMotor.config_kP(Constants.kVelocityPIDSlotIdx0, ClimbConstants.kP_front, Constants.kTimeoutMs);
        m_frontClimbMotor.config_kI(Constants.kVelocityPIDSlotIdx0, ClimbConstants.kI_front, Constants.kTimeoutMs);
        m_frontClimbMotor.config_kD(Constants.kVelocityPIDSlotIdx0, ClimbConstants.kD_front, Constants.kTimeoutMs);
        m_frontClimbMotor.config_IntegralZone(Constants.kVelocityPIDSlotIdx0, ClimbConstants.kFrontIzone);
        m_frontClimbMotor.configClosedloopRamp(ClimbConstants.kClosedLoopRamp, Constants.kTimeoutMs);
        m_frontClimbMotor.selectProfileSlot(Constants.kVelocityPIDSlotIdx0, Constants.kPidIdx0);

        // Configure rear motor
        m_rearClimbMotor = new WPI_TalonSRX(ClimbConstants.kRearClimbMotorPort);
        m_rearClimbMotor.configFactoryDefault();
        // Set rear motor limits
        m_rearClimbMotor.configForwardSoftLimitThreshold(ClimbConstants.kRearForwardSoftLimit, 0);
        m_rearClimbMotor.configReverseSoftLimitThreshold(ClimbConstants.kRearReverseSoftLimit, 0);
        m_rearClimbMotor.configForwardSoftLimitEnable(true, 0);
        m_rearClimbMotor.configReverseSoftLimitEnable(true, 0);
        m_rearClimbMotor.setNeutralMode(NeutralMode.Brake);
        m_rearClimbMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kVelocityPIDSlotIdx0, Constants.kTimeoutMs);
        m_rearClimbMotor.setInverted(ClimbConstants.kRearInvertMotor);
        m_rearClimbMotor.setSensorPhase(ClimbConstants.kRearInvertSensorDirection);
        m_rearClimbMotor.config_kF(Constants.kVelocityPIDSlotIdx0, ClimbConstants.kF_rear, Constants.kTimeoutMs);
        m_rearClimbMotor.config_kP(Constants.kVelocityPIDSlotIdx0, ClimbConstants.kP_rear, Constants.kTimeoutMs);
        m_rearClimbMotor.config_kI(Constants.kVelocityPIDSlotIdx0, ClimbConstants.kI_rear, Constants.kTimeoutMs);
        m_rearClimbMotor.config_kD(Constants.kVelocityPIDSlotIdx0, ClimbConstants.kD_rear, Constants.kTimeoutMs);
        m_rearClimbMotor.config_IntegralZone(Constants.kVelocityPIDSlotIdx0, ClimbConstants.kRearIzone);
        m_rearClimbMotor.configClosedloopRamp(ClimbConstants.kClosedLoopRamp, Constants.kTimeoutMs);
        m_rearClimbMotor.selectProfileSlot(Constants.kVelocityPIDSlotIdx0, Constants.kPidIdx0);

        m_climberSolenoid = new Solenoid(ClimbConstants.kClimberSolenoid);
        m_gyro = new AHRS(SPI.Port.kMXP);

        resetGyro();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        log();
    }

    /**
     * Get the pitch -180 to 180
     * @return the pitch
     */
    public double getTilt() {
        return m_gyro.getRoll();
    }

    public void runFrontMotorAtVelocity(double velocity) {
        // Safety check to make sure the solenoid is disengaged (true)
//        if (m_climberSolenoid.get()) { // TODO: fix this
            m_frontClimbMotor.set(ControlMode.Velocity, velocity);
//        }
    }

    public void runRearMotorAtVelocity(double velocity) {
        // Safety check to make sure the solenoid is disengaged (true)
//        if (m_climberSolenoid.get()) {
            m_rearClimbMotor.set(ControlMode.Velocity, velocity);
//        }
    }

    /**
     * Get the rear encoder position in raw sensor units
     * @return raw sensor units
     */
    public int getRearPositionTics(){
        return m_rearClimbMotor.getSelectedSensorPosition();
    }

    /**
     * Get the front encoder position in raw sensor units
     * @return raw sensor units
     */
    public int getFrontPositionTics(){
        return m_frontClimbMotor.getSelectedSensorPosition();
    }

    public void engageClimberSolenoid() {
        m_climberSolenoid.set(false);
    }

    public void disengageClimberSolenoid() {
        m_climberSolenoid.set(true);
    }

    /**
     * Resets the heading of the robot
     */
    public void resetGyro() {
        m_gyro.reset();
        m_gyro.zeroYaw();
    }

    public boolean getSolenoidState(){
        return m_climberSolenoid.get();
    }

    public boolean solenoidDisengaged() {
        return m_climberSolenoid.get();
    }

    public void disableReverseSoftLimits() {
        m_frontClimbMotor.configReverseSoftLimitEnable(false, 0);
        m_rearClimbMotor.configReverseSoftLimitEnable(false, 0);
    }

    public void enableReverseSoftLimits() {
        m_frontClimbMotor.configReverseSoftLimitEnable(true, 0);
        m_rearClimbMotor.configReverseSoftLimitEnable(true, 0);
    }

    public void log() {
        SmartDashboard.putBoolean("Climber Solenoid Engaged", m_climberSolenoid.get());
        SmartDashboard.putNumber("Angle of Tilt", getTilt());
    }
}
