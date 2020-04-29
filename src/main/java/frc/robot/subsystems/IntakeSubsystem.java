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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.utils.Utils;

public class IntakeSubsystem extends SubsystemBase {
  private final WPI_TalonSRX m_motor;
  private final Solenoid m_intakeSolenoid;

  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {

    m_motor = new WPI_TalonSRX(IntakeConstants.kIntakeMotor);
    m_motor.configFactoryDefault();
    m_motor.setNeutralMode(NeutralMode.Coast);
    m_motor.setInverted(IntakeConstants.kInvertMotor);
    m_motor.setSensorPhase(IntakeConstants.kInvertSensorDirection);
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kVelocityPIDSlotIdx0, Constants.kTimeoutMs);
    m_motor.config_kF(Constants.kVelocityPIDSlotIdx0, IntakeConstants.kF, Constants.kTimeoutMs);
    m_motor.config_kP(Constants.kVelocityPIDSlotIdx0, IntakeConstants.kP, Constants.kTimeoutMs);
    m_motor.config_kI(Constants.kVelocityPIDSlotIdx0, IntakeConstants.kI, Constants.kTimeoutMs);
    m_motor.config_kD(Constants.kVelocityPIDSlotIdx0, IntakeConstants.kD, Constants.kTimeoutMs);
    m_motor.config_IntegralZone(Constants.kVelocityPIDSlotIdx0, IntakeConstants.kIzone);
    m_motor.configClosedloopRamp(IntakeConstants.kClosedLoopRamp, Constants.kTimeoutMs);
    m_motor.selectProfileSlot(Constants.kVelocityPIDSlotIdx0, Constants.kPidIdx0);


    m_intakeSolenoid = new Solenoid(IntakeConstants.kIntakeSolenoid);
    // Set the default speed of the intake when enabled

  }

  public void runIntakeMotor(double speed) {
    m_motor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Run the motor at a set velocity
   *
   * @param velocity in raw tics / 100ms
   */
  public void runIntakeMotorAtVelocity(double velocity) {
    m_motor.set(ControlMode.Velocity, velocity);
  }

  /**
   * Toggle the intake manipulator in and out
   */
  public void toggleIntakeManipulator() {
    m_intakeSolenoid.set(!m_intakeSolenoid.get());
  }

  /**
   * Deploys the intake manipulator
   */
  public void deployIntake() {
    m_intakeSolenoid.set(true);
  }

  /**
   * Retracts the intake manipulator
   */
  public void retractIntake() {
    m_intakeSolenoid.set(false);
  }

  /**
   * Check the current state of the intake manipulator
   * 
   * @return True if the manipulator is deployed
   */
  public boolean intakeDeployed() {
    return m_intakeSolenoid.get();
  }

  @Override
  public void periodic() {
    log();
  }

  public void log() {
    SmartDashboard.putNumber("Intake RPS",
        Utils.tics2Rps(m_motor.getSelectedSensorVelocity(Constants.kVelocityPIDSlotIdx0),
            1,
            IntakeConstants.kPPR));
  }
}
