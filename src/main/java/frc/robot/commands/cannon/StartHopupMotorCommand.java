package frc.robot.commands.cannon;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CannonSubsystem;


public class StartHopupMotorCommand extends CommandBase {
  private final CannonSubsystem m_subsystem;

  public StartHopupMotorCommand(CannonSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.runHopupMotorAtVelocity(Constants.CannonConstants.kHopupMotorVelocity);
  }

  @Override
  public void execute() {

  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
//      m_subsystem.runHopupMotorAtVelocity(0);
  }
}
