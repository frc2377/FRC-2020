package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.Constants;

public class TalonSRXEncoder implements Sendable {

  private final TalonSRX talon;

  private double distancePerPulse = 1;

  public TalonSRXEncoder(TalonSRX talon) {
    this.talon = talon;
  }
  public void setDistancePerPulse(double distancePerPulse) {
    this.distancePerPulse = distancePerPulse;
  }

  public void reset() {
    talon.setSelectedSensorPosition(0);
  }

  public double getDistancePerPulse() {
    return distancePerPulse;
  }

  public double getDistance() {
    return talon.getSelectedSensorPosition(Constants.kVelocityPIDSlotIdx0) * distancePerPulse;
  }

  public double getRate() {
    return talon.getSelectedSensorVelocity(Constants.kVelocityPIDSlotIdx0) * distancePerPulse;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Encoder");
    builder.addDoubleProperty("Distance", this::getDistance, null);
    builder.addDoubleProperty("Speed", this::getRate, null);
    builder.addDoubleProperty("Distance per tick", this::getDistancePerPulse, null);
  }
}
