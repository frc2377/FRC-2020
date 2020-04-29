/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.utils.Magazine;


public class ConveyorSubsystemDefaultCommand extends CommandBase {
  private final ConveyorSubsystem m_subsystem;

  public ConveyorSubsystemDefaultCommand(ConveyorSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
      m_subsystem = subsystem;
      addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Magazine.numberOfPowerCells() < 5) {    // Determine if any space remains in conveyor
      if (Magazine.sensor1Blocked()) {       // A Power Cell is in position 1
        Timer.delay(0.025);
        m_subsystem.advanceConveyorOnePowercell();           // Move the power cell to the next slot
      }
    } else {
      m_subsystem.runMotorAtVelocity(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}