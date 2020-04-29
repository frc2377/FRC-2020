/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// This code is based on the example in the WPILib Java Examle for gearsbot

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class TankDriveCommand extends CommandBase {
    private final DriveSubsystem m_drivetrain;
    private final DoubleSupplier m_left;
    private final DoubleSupplier m_right;

    /**
     * Creates a new TankDrive command.
     *
     * @param left       The control input for the left side of the drive
     * @param right      The control input for the right sight of the drive
     * @param drivetrain The drivetrain subsystem to drive
     */
    public TankDriveCommand(final DoubleSupplier left, final DoubleSupplier right, final DriveSubsystem drivetrain) {
        m_drivetrain = drivetrain;
        m_left = left;
        m_right = right;
        addRequirements(m_drivetrain);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drivetrain.drive(-m_left.getAsDouble(),
                -m_right.getAsDouble());
        log();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
        m_drivetrain.drive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private void log() {
        SmartDashboard.putNumber("Left Joystick", m_left.getAsDouble());
        SmartDashboard.putNumber("Right Joystick", m_right.getAsDouble());
    }
}
