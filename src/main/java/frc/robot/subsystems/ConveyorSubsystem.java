package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.utils.Magazine;

public class ConveyorSubsystem extends SubsystemBase {
  private final WPI_TalonSRX m_motor;

  public ConveyorSubsystem() {

    m_motor = new WPI_TalonSRX(ConveyorConstants.kConveyorMotor);
    m_motor.configFactoryDefault();
    m_motor.configContinuousCurrentLimit(25);
    m_motor.setNeutralMode(NeutralMode.Coast);
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kVelocityPIDSlotIdx0, Constants.kTimeoutMs);
    m_motor.setInverted(ConveyorConstants.kInvertMotor);
    m_motor.setSensorPhase(ConveyorConstants.kInvertSensorDirection);

    // Configure PID Slot 0 for velocity
    m_motor.config_kF(Constants.kVelocityPIDSlotIdx0, ConveyorConstants.kF_vel, Constants.kTimeoutMs);
    m_motor.config_kP(Constants.kVelocityPIDSlotIdx0, ConveyorConstants.kP_vel, Constants.kTimeoutMs);
    m_motor.config_kI(Constants.kVelocityPIDSlotIdx0, ConveyorConstants.kI_vel, Constants.kTimeoutMs);
    m_motor.config_kD(Constants.kVelocityPIDSlotIdx0, ConveyorConstants.kD_vel, Constants.kTimeoutMs);
    m_motor.config_IntegralZone(Constants.kVelocityPIDSlotIdx0, ConveyorConstants.kIzone);
    m_motor.configClosedloopRamp(ConveyorConstants.kClosedLoopRamp, Constants.kTimeoutMs);

    // Configure PID Slot 1 for position
    m_motor.config_kP(Constants.kPositionPIDSlotIdx1, ConveyorConstants.kP_pos, Constants.kTimeoutMs);
    m_motor.config_kI(Constants.kPositionPIDSlotIdx1, ConveyorConstants.kI_pos, Constants.kTimeoutMs);
    m_motor.config_kD(Constants.kPositionPIDSlotIdx1, ConveyorConstants.kD_pos, Constants.kTimeoutMs);
    m_motor.config_IntegralZone(Constants.kPositionPIDSlotIdx1, ConveyorConstants.kIzone);
    m_motor.configClosedloopRamp(ConveyorConstants.kClosedLoopRamp, Constants.kTimeoutMs);

    resetEncoder();
  }

  /**
   * Reset the conveyor encoder position to 0
   */
  public void resetEncoder() {
    m_motor.setSelectedSensorPosition(0, Constants.kVelocityPIDSlotIdx0, Constants.kTimeoutMs);
  }

  /**
   * Returns the current position in raw tics
   * @return raw tics of current position
   */
  public int getCurrentPosition() {
    return m_motor.getSelectedSensorPosition(Constants.kVelocityPIDSlotIdx0);
  }

  /**
   * Run the conveyor at a velocity measured in raw tics per 100ms
   * @param velocity velocity in raw tics per 100ms
   */
  public void runMotorAtVelocity(double velocity) {
    selectProfileSlot(Constants.kVelocityPIDSlotIdx0);
    m_motor.set(ControlMode.Velocity, velocity);
  }

  /**
   * Run the conveyor to a position
   * @param position position in raw tics
   */
  public void runConveyorToPosition(int position) {
    selectProfileSlot(Constants.kPositionPIDSlotIdx1);
    m_motor.set(ControlMode.Position, position);
  }

  /**
   * Advance the conveyor 1 Power Cell
   */
  public void advanceConveyorOnePowercell() {
    runConveyorToPosition(getCurrentPosition() + ConveyorConstants.kAdvanceOnePowerCellDistance);
  }

  public boolean conveyorMotorAtSpeed() {
    return Math.abs(m_motor.getClosedLoopError(Constants.kVelocityPIDSlotIdx0)) < ConveyorConstants.kVelocityTolerance;
  }

  /**
   * Select the PID corresponding with Velocity (0) or Position (1)
   * @param slotIdx 0 for Velocity, 1 for Position
   */
  public void selectProfileSlot(int slotIdx) {
    m_motor.selectProfileSlot(slotIdx, Constants.kPidIdx0);
  }

  @Override
  public void periodic() {
    log();
  }

  public void log() {
    SmartDashboard.putNumber("Conveyor Position", getCurrentPosition());
    SmartDashboard.putNumber("Conveyor Speed", m_motor.getSelectedSensorVelocity(Constants.kVelocityPIDSlotIdx0));
    SmartDashboard.putBoolean("Conveyor at Speed", conveyorMotorAtSpeed());
    SmartDashboard.putNumber("Conveyor Closed Loop Error", m_motor.getClosedLoopError(Constants.kVelocityPIDSlotIdx0));
    SmartDashboard.putNumber("PC in Conveyor Count", Magazine.numberOfPowerCells());
    showSensors();
  }

  public void showSensors() {
    SmartDashboard.putBoolean("Sensor 1 Blocked", Magazine.sensor1Blocked());
    SmartDashboard.putBoolean("Sensor 2 Blocked", Magazine.sensor2Blocked());
    SmartDashboard.putBoolean("Sensor 3 Blocked", Magazine.sensor3Blocked());
    SmartDashboard.putBoolean("Sensor 4 Blocked", Magazine.sensor4Blocked());
    SmartDashboard.putBoolean("Sensor 5 Blocked", Magazine.sensor5Blocked());
  }


}
