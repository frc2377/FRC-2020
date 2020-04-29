/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {

  private final WPI_TalonSRX m_motor;

  /**
   * Creates a new TurretSubsystem.
   */
  public TurretSubsystem() {
    m_motor = new WPI_TalonSRX(TurretConstants.kTurretMotorPort);

    m_motor.configFactoryDefault();
    m_motor.configForwardSoftLimitThreshold(TurretConstants.kSoftLimit, 0);
    m_motor.configReverseSoftLimitThreshold(-TurretConstants.kSoftLimit, 0);
    m_motor.configForwardSoftLimitEnable(true, 0);
    m_motor.configReverseSoftLimitEnable(true, 0);

    m_motor.setNeutralMode(NeutralMode.Brake);
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPositionPIDSlotIdx1, Constants.kTimeoutMs);
    m_motor.setSensorPhase(TurretConstants.kInvertSensorDirection);
    m_motor.setInverted(TurretConstants.kInvertMotor);
    m_motor.config_kF(Constants.kPositionPIDSlotIdx1, TurretConstants.kF, Constants.kTimeoutMs);
    m_motor.config_kP(Constants.kPositionPIDSlotIdx1, TurretConstants.kP, Constants.kTimeoutMs);
    m_motor.config_kI(Constants.kPositionPIDSlotIdx1, TurretConstants.kI, Constants.kTimeoutMs);
    m_motor.config_kD(Constants.kPositionPIDSlotIdx1, TurretConstants.kD, Constants.kTimeoutMs);
    m_motor.config_IntegralZone(Constants.kPositionPIDSlotIdx1, TurretConstants.kIzone);
    m_motor.configClosedloopRamp(TurretConstants.kClosedLoopRamp, Constants.kTimeoutMs);
    m_motor.selectProfileSlot(Constants.kPositionPIDSlotIdx1, Constants.kPidIdx0);

    resetEncoder();
  }

  /**
   * Reset turret encoder position to 0
   */
  public void resetEncoder() {
    m_motor.setSelectedSensorPosition(0);
  }

  public int getCurrentPosition() {
    return m_motor.getSelectedSensorPosition(Constants.kVelocityPIDSlotIdx0); }

  public void runMotor(double speed, int scale) {
    if (scale == 0) {
      m_motor.set(ControlMode.PercentOutput, 0);
    } else {
      m_motor.set(ControlMode.PercentOutput, speed / scale);
    }
  }

  public void runMotor(double speed) {
    runMotor(speed, 1);
  }

  public void turnToPosition(int position) {
    m_motor.set(ControlMode.Position, position/*, DemandType.ArbitraryFeedForward, TurretConstants.kArbitraryFeedForward*/);
  }

  @Override
  public void periodic() {
    log();
  }

  public void log() {
  }
}
