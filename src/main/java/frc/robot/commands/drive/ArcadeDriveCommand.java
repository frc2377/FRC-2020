/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class ArcadeDriveCommand extends CommandBase {
    private final DriveSubsystem m_drivetrain;
    private final DoubleSupplier m_fwdAxis;
    private final DoubleSupplier m_rotAxis;

    /**
     * Creates a new ArcadeDrive.
     *
     * @param m_fwdAxis  The control input for fwd/back controlled by the left stick
     * @param m_rotAxis  The control input for turning controlled by the right stick
     * @param drivetrain The drivetrain subsystem to drive
     */
    public ArcadeDriveCommand(final DoubleSupplier fwdAxis, final DoubleSupplier rotAxis, final DriveSubsystem drivetrain) {
        m_drivetrain = drivetrain;
        m_fwdAxis = fwdAxis;
        m_rotAxis = rotAxis;
        addRequirements(m_drivetrain);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drivetrain.arcadeDrive(m_fwdAxis.getAsDouble(), m_rotAxis.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
        m_drivetrain.arcadeDrive(0.0, 0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
