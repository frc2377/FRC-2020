/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*  coded by : Quentin Molina                                                                          */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.manipulatecontrolpanel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ManipulateControlPanelConstants;
import frc.robot.subsystems.ManipulateControlPanelSubsystem;
import frc.robot.subsystems.ManipulateControlPanelSubsystem.PPColor;

public class AutoRotateStage2Command extends CommandBase {
    ManipulateControlPanelSubsystem m_subsystem;
    private int m_colorCount;
    private PPColor m_currentColor;
    private PPColor m_previousColor;

    private boolean m_armRaised = false;

    /**
     * Creates a new RotateStage2Command.
     */
    public AutoRotateStage2Command(ManipulateControlPanelSubsystem subsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_subsystem = subsystem;
        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Ensure the arm is raised
        m_armRaised = m_subsystem.controlPanelArmDeployed();
        if (m_armRaised) {
            m_colorCount = 0;
            m_currentColor = m_subsystem.readColorSensor();
            m_previousColor = m_currentColor;
            // Put the initial state on the dashboard
            upateDashboard();

            // Start rotating the motor
            m_subsystem.rotatePowerPanelMotor(ManipulateControlPanelConstants.kControlPanelMotorVelocity);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_armRaised) {
            m_currentColor = m_subsystem.readColorSensor();
            // Count every time the color changes
            if (m_currentColor != PPColor.Unknown && m_currentColor == getExpectedNextColor(m_previousColor)) {
                m_colorCount++;
                m_previousColor = m_currentColor;
            }
            // Update the Dashboard
            upateDashboard();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stop rotating
        m_subsystem.stopPowerPanelMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // End the command if the sensor reads unknown
        //
        return !m_armRaised ||
            m_currentColor == PPColor.Unknown ||
            m_colorCount >= ManipulateControlPanelConstants.kStage2ColorCount;
    }

    private void upateDashboard() {
        SmartDashboard.putString("Current Color", m_currentColor.toString());
        SmartDashboard.putString("Previous Color", m_previousColor.toString());
        SmartDashboard.putNumber("Current Count", m_colorCount);
    }

    // Code Below only works when Color Wheel is spun Counter-Clockwise.
    public PPColor getExpectedNextColor(PPColor currentColor) {
        PPColor nextColor = PPColor.Unknown;

        if (currentColor == PPColor.Yellow) {
            nextColor = PPColor.Blue;
        } else if (currentColor == PPColor.Blue) {
            nextColor = PPColor.Green;
        } else if (currentColor == PPColor.Green) {
            nextColor = PPColor.Red;
        } else if (currentColor == PPColor.Red) {
            nextColor = PPColor.Yellow;
        }
        return nextColor;
    }
}
