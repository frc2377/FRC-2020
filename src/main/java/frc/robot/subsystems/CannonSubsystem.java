/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.

*Keenan the leprechaun made this PID code!/
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CannonConstants;
import frc.robot.utils.Limelight;


public class CannonSubsystem extends SubsystemBase {
  /**
   * Creates a new CannonSubsystem.
   */

  private final WPI_TalonSRX m_upperMotor;
  private final WPI_TalonSRX m_lowerMotor;
  private final WPI_TalonSRX m_hopUpMotor;


  //Creates a new CannonSubsystem
  public CannonSubsystem() {
    m_upperMotor = new WPI_TalonSRX(CannonConstants.kUpperCannonMotorPort);
    m_upperMotor.configFactoryDefault();
    m_upperMotor.setNeutralMode(NeutralMode.Coast);
    m_upperMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kVelocityPIDSlotIdx0, Constants.kTimeoutMs);
    m_upperMotor.setSensorPhase(CannonConstants.kUpperInvertSensorDirection);
    m_upperMotor.setInverted(CannonConstants.kUpperInvertMotor);
    m_upperMotor.config_kF(Constants.kVelocityPIDSlotIdx0, CannonConstants.kF_upper, Constants.kTimeoutMs);
    m_upperMotor.config_kP(Constants.kVelocityPIDSlotIdx0, CannonConstants.kP_upper, Constants.kTimeoutMs);
    m_upperMotor.config_kI(Constants.kVelocityPIDSlotIdx0, CannonConstants.kI_upper, Constants.kTimeoutMs);
    m_upperMotor.config_kD(Constants.kVelocityPIDSlotIdx0, CannonConstants.kD_upper, Constants.kTimeoutMs);
    m_upperMotor.config_IntegralZone(Constants.kVelocityPIDSlotIdx0, CannonConstants.kUpperIzone);
    m_upperMotor.configClosedloopRamp(CannonConstants.kClosedLoopRamp, Constants.kTimeoutMs);
    m_upperMotor.selectProfileSlot(Constants.kVelocityPIDSlotIdx0, Constants.kPidIdx0);

    m_lowerMotor = new WPI_TalonSRX(CannonConstants.kLowerCannonMotorPort);
    m_lowerMotor.configFactoryDefault();
    m_lowerMotor.setNeutralMode(NeutralMode.Coast);
    m_lowerMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kVelocityPIDSlotIdx0, Constants.kTimeoutMs);
    m_lowerMotor.setSensorPhase(CannonConstants.kLowerInvertSensorDirection);
    m_lowerMotor.setInverted(CannonConstants.kLowerInvertMotor);
    m_lowerMotor.config_kF(Constants.kVelocityPIDSlotIdx0, CannonConstants.kF_lower, Constants.kTimeoutMs);
    m_lowerMotor.config_kP(Constants.kVelocityPIDSlotIdx0, CannonConstants.kP_lower, Constants.kTimeoutMs);
    m_lowerMotor.config_kI(Constants.kVelocityPIDSlotIdx0, CannonConstants.kI_lower, Constants.kTimeoutMs);
    m_lowerMotor.config_kD(Constants.kVelocityPIDSlotIdx0, CannonConstants.kD_lower, Constants.kTimeoutMs);
    m_lowerMotor.config_IntegralZone(Constants.kVelocityPIDSlotIdx0, CannonConstants.kLowerIzone);
    m_lowerMotor.configClosedloopRamp(CannonConstants.kClosedLoopRamp, Constants.kTimeoutMs);
    m_lowerMotor.selectProfileSlot(Constants.kVelocityPIDSlotIdx0, Constants.kPidIdx0);


    m_hopUpMotor = new WPI_TalonSRX(CannonConstants.kHopupMotorPort);
    m_hopUpMotor.configFactoryDefault();
    m_hopUpMotor.setNeutralMode(NeutralMode.Coast);
    m_hopUpMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kVelocityPIDSlotIdx0, Constants.kTimeoutMs);
    m_hopUpMotor.setInverted(CannonConstants.kHopupInvertMotor);
    m_hopUpMotor.setSensorPhase(CannonConstants.kHopupInvertSensorDirection);
    m_hopUpMotor.config_kF(Constants.kVelocityPIDSlotIdx0, CannonConstants.kF_hopup, Constants.kTimeoutMs);
    m_hopUpMotor.config_kP(Constants.kVelocityPIDSlotIdx0, CannonConstants.kP_hopup, Constants.kTimeoutMs);
    m_hopUpMotor.config_kI(Constants.kVelocityPIDSlotIdx0, CannonConstants.kI_hopup, Constants.kTimeoutMs);
    m_hopUpMotor.config_kD(Constants.kVelocityPIDSlotIdx0, CannonConstants.kD_hopup, Constants.kTimeoutMs);
    m_hopUpMotor.config_IntegralZone(Constants.kVelocityPIDSlotIdx0, CannonConstants.kHopupIzone);
    m_hopUpMotor.configClosedloopRamp(CannonConstants.kClosedLoopRamp, Constants.kTimeoutMs);
    m_hopUpMotor.selectProfileSlot(Constants.kVelocityPIDSlotIdx0, Constants.kPidIdx0);

  }

  public void runCannonMotorsAtVelocity(double lower, double upper) {
    m_upperMotor.set(ControlMode.Velocity, upper);
    m_lowerMotor.set(ControlMode.Velocity, lower);
  }

  public void runHopupMotorAtVelocity(double velocity) {
    m_hopUpMotor.set(ControlMode.Velocity, velocity);
  }

  /**
   * If range is < 6, return kFiringTable 0
   * If range is 6-7, return kFiringTable 1
   * If range is 7-10, return kFiringTable 2
   * If range is 10-15, return kFiringTable 3
   * If range is 15-20, return kFiringTable 4
   * If range is 20-25, return kFiringTable 5
   *
   * If range is > 25 feet (out of range), fire at full power
   *
   * @param range the distance provided by the Limelight
   * @return an integer array container [lowerSetpoint, upperSetpoint]
   */
  public int[] getSetpoints(double range) {

    // If range is < 6, return kFiringTable 0
    // If range is 6-7, return kFiringTable 1
    // if range is 7-10, return kFiringTable 2
    // if range is 10-15, return kFiringTable 3
    // If range is 15-20, return kFiringTable 4
    // if range is 20-25, return kFiringTable 5

    // If range is > 25 feet (out of range), fire at full power

    if (range < 6) {
      return CannonConstants.kTacticalTargetingTable[0];
    } else if (range >= 6 && range <= 7) {
      return CannonConstants.kTacticalTargetingTable[1];
    } else if (range > 7 && range <= 10) {
      return CannonConstants.kTacticalTargetingTable[2];
    } else if (range > 10 && range <= 15) {
      return CannonConstants.kTacticalTargetingTable[3];
    } else if (range > 15 && range <= 20) {
      return CannonConstants.kTacticalTargetingTable[4];
    } else if (range > 20 && range <= 25) {
      return CannonConstants.kTacticalTargetingTable[5];
    }

    // If out of range, fire at full power
    return CannonConstants.kTacticalTargetingTable[0];
  }

  @Override
  public void periodic() {
    log();
  }

  public void log() {
    SmartDashboard.putNumber("Upper Cannon Speed", m_upperMotor.getSelectedSensorVelocity(Constants.kVelocityPIDSlotIdx0));
    SmartDashboard.putNumber("Lower Cannon Speed", m_lowerMotor.getSelectedSensorVelocity(Constants.kVelocityPIDSlotIdx0));
    SmartDashboard.putNumber("Upper Cannon Closed Loop Error", m_upperMotor.getClosedLoopError(Constants.kVelocityPIDSlotIdx0));
    SmartDashboard.putNumber("Lower Cannon Closed Loop Error", m_lowerMotor.getClosedLoopError(Constants.kVelocityPIDSlotIdx0));
    SmartDashboard.putBoolean("Limelight has target", Limelight.hasTarget());
    SmartDashboard.putNumber("Hopup Closed Loop Error", m_hopUpMotor.getClosedLoopError(Constants.kVelocityPIDSlotIdx0));
    SmartDashboard.putNumber("Hopup Speed", m_hopUpMotor.getSelectedSensorVelocity(Constants.kVelocityPIDSlotIdx0));
  }
}