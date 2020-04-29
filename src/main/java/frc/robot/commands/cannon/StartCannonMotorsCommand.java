/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cannon;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.utils.Limelight;

public class StartCannonMotorsCommand extends CommandBase {
  private final CannonSubsystem m_subsystem;

  /**
   * Creates a new StartCannonMotors.
   */
  public StartCannonMotorsCommand(final CannonSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double distance = 10;
    if (Limelight.hasTarget()) {
      distance = Limelight.getDistance();
    }
    int[] setpoints = m_subsystem.getSetpoints(distance);
    m_subsystem.runCannonMotorsAtVelocity(setpoints[0], setpoints[1]);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
//    m_subsystem.runCannonMotorsAtVelocity(0,0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
