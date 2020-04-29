 /*----------------------------------------------------------------------------*/
 /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
 /* Open Source Software - may be modified and shared by FRC teams. The code   */
 /* must be accompanied by the FIRST BSD license file in the root directory of */
 /* the project.                                                               */
 /*----------------------------------------------------------------------------*/

 package frc.robot.commands.intake;

 import edu.wpi.first.wpilibj2.command.CommandBase;
 import frc.robot.Constants.IntakeConstants;
 import frc.robot.subsystems.IntakeSubsystem;
 import frc.robot.utils.Magazine;

 public class DeployIntakeCommand extends CommandBase {
     final private IntakeSubsystem m_intakeSubsystem;

     /**
      * Creates a new Deploy.
      */
     public DeployIntakeCommand(IntakeSubsystem subsystem) {
         // Use addRequirements() here to declare subsystem dependencies.
         m_intakeSubsystem = subsystem;
         addRequirements(subsystem);
     }

     // Called when the command is initially scheduled.
     @Override
     public void initialize() {
         if (!m_intakeSubsystem.intakeDeployed()) {
             m_intakeSubsystem.deployIntake();
         }
     }

     // Called every time the scheduler runs while the command is scheduled.
     @Override
     public void execute() {
         // Stop the motors if the intake is full, continue once there is room
         if (Magazine.magazineIsFull()) {
             m_intakeSubsystem.runIntakeMotor(0);
         } else {
            m_intakeSubsystem.runIntakeMotor(IntakeConstants.kMotorSpeed);
         }
     }

     // Called once the command ends or is interrupted.
     @Override
     public void end(boolean interrupted) {
     }

     // Returns true when the command should end.
     @Override
     public boolean isFinished() {
         return false;
     }
 }
