package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;


public class DeployArmsCommand extends CommandBase {
  private final ClimbSubsystem climbSubsystem;

  public DeployArmsCommand(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    addRequirements(climbSubsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize() {
    climbSubsystem.disengageClimberSolenoid();
    Timer.delay(0.5);
    climbSubsystem.runFrontMotorAtVelocity(Constants.ClimbConstants.kFrontDeploySpeed);
    climbSubsystem.runRearMotorAtVelocity(Constants.ClimbConstants.kRearDeploySpeed);
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled.
   * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute() {

  }

  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes -- indicated by
   * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
   * </p><p>
   * Returning false will result in the command never ending automatically. It may still be
   * cancelled manually or interrupted by another command. Hard coding this command to always
   * return true will result in the command executing once and finishing immediately. It is
   * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
   * for such an operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  /**
   * The action to take when the command ends. Called when either the command
   * finishes normally -- that is it is called when {@link #isFinished()} returns
   * true -- or when  it is interrupted/canceled. This is where you may want to
   * wrap up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {

    climbSubsystem.runRearMotorAtVelocity(0);
    climbSubsystem.runFrontMotorAtVelocity(0);
    Timer.delay(0.5);
    climbSubsystem.engageClimberSolenoid();
  }
}
