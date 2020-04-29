package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class StopShootingCommand extends SequentialCommandGroup {

  public StopShootingCommand(CannonSubsystem cannon, ConveyorSubsystem conveyor, IntakeSubsystem intake) {
    addCommands(
        new InstantCommand(() -> cannon.runCannonMotorsAtVelocity(0, 0), cannon),
        new InstantCommand(() -> cannon.runHopupMotorAtVelocity(0), cannon),
        new InstantCommand(() -> conveyor.runMotorAtVelocity(0), conveyor),
        new InstantCommand(() -> intake.runIntakeMotor(0), intake)
    );
  }
}