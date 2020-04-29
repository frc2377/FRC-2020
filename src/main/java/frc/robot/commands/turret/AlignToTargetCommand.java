/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.Limelight;
import frc.robot.utils.Utils;

public class AlignToTargetCommand extends CommandBase {

    private final TurretSubsystem m_subsystem;

    private boolean hasTarget;
    private boolean validTarget = true;

    /**
     * Creates a new AlignToTarget.
     */
    public AlignToTargetCommand(TurretSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(m_subsystem);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    // TODO: Future work.  Add a degree range for valid target so that we don't align to the wrong target
    public void initialize() {
        Limelight.enableLED();
        SmartDashboard.putString("Turret Auto Status", "Turret is Auto Aligning");
        SmartDashboard.putString("Turret Auto Status Interrupted", "nm,.");
        hasTarget = Limelight.hasTarget();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        hasTarget = Limelight.hasTarget();
        if (hasTarget) {
            double tx = Limelight.getTx();
            int tx_tics = Utils.degrees2Tics(tx, Constants.TurretConstants.kGearRatio, Constants.TurretConstants.kPPR);
            int currentPos = m_subsystem.getCurrentPosition();
            m_subsystem.turnToPosition(currentPos + tx_tics);
        }
        updateDashboard(); // TODO: Remove this once turret is tuned
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Limelight.disableLED();
        SmartDashboard.putString("Turret Auto Status", "Turret stopped Auto Aligning");
        if (interrupted) {
            SmartDashboard.putString("Turret Auto Status Interrupted", "Turret Auto Aligning Interrupted");
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private void updateDashboard() {
        SmartDashboard.putNumber("Turret Current Position", m_subsystem.getCurrentPosition());
        SmartDashboard.putNumber("Limelight tx", Limelight.getTx());
        SmartDashboard.putBoolean("Limelight has target", Limelight.hasTarget());
    }
}
