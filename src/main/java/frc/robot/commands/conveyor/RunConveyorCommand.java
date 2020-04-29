package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.ConveyorSubsystem;


public class RunConveyorCommand extends CommandBase {
  private final ConveyorSubsystem m_subsystem;

  public RunConveyorCommand(ConveyorSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize() {
    m_subsystem.runMotorAtVelocity(ConveyorConstants.kConveyorMotorVelocity);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {

  }
}
