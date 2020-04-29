/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.cannon.StartCannonMotorsCommand;
import frc.robot.commands.conveyor.RunConveyorCommand;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootPowerCellsCommand extends SequentialCommandGroup {
  /**
   * Creates a new ShootPowerCellsCommand.
   */
  public ShootPowerCellsCommand(CannonSubsystem cannon, ConveyorSubsystem conveyor, IntakeSubsystem intake) {
    addCommands(
        new StartCannonMotorsCommand(cannon),
        new InstantCommand(() -> Timer.delay(0.3)),
        new InstantCommand(()->cannon.runHopupMotorAtVelocity(Constants.CannonConstants.kHopupMotorVelocity), cannon),
        new InstantCommand(() -> Timer.delay(0.2)),
        new RunConveyorCommand(conveyor),
        new InstantCommand(intake::deployIntake, intake),
        new InstantCommand(() -> Timer.delay(0.1)),
        new InstantCommand(() -> intake.runIntakeMotor(Constants.IntakeConstants.kMotorSpeed), intake)
    );
  }
}