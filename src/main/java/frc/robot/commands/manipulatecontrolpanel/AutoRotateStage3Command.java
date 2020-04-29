/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.manipulatecontrolpanel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ManipulateControlPanelSubsystem;
import frc.robot.subsystems.ManipulateControlPanelSubsystem.PPColor;

;

public class AutoRotateStage3Command extends CommandBase {
    ManipulateControlPanelSubsystem m_subsystem;

    private PPColor m_currentColor;
    private PPColor m_allianceColor;
    private PPColor m_correctedCurrentColor;

    private boolean m_armedRaised = false;

    /**
     * Creates a new RotateStage3Command.
     */
    public AutoRotateStage3Command(ManipulateControlPanelSubsystem subsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_subsystem = subsystem;
        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Ensure the arm is deployed
        m_armedRaised = m_subsystem.controlPanelArmDeployed();
        if (m_armedRaised) {
            // Read the current Color
            m_currentColor = m_subsystem.readColorSensor();
            m_correctedCurrentColor = m_currentColor;
            // Get the alliance Color
            m_allianceColor = getStage3AllianceColor();

            m_subsystem.releaseMotor();

            // Start rotating if the color isn't correct
            if (calculateFieldSensorColor(m_currentColor) != m_allianceColor) {
                m_subsystem.rotatePowerPanelMotor(0.1);
            }
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_armedRaised) {
            m_currentColor = m_subsystem.readColorSensor();
            if (m_currentColor != PPColor.Unknown && m_currentColor == getExpectedNextColor(m_correctedCurrentColor)) {
                m_correctedCurrentColor = m_currentColor;
            }
            updateDashboard();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stop the wheel
        m_subsystem.rotatePowerPanelMotor(0);
        m_subsystem.brakeMotor();
    }

    /**
     * Returns true when the command should end.
     *
     * @return Ends when the current color matches the alliance color, the alliance
     *         color is unknown, or the sensor reads unknown
     */
    @Override
    public boolean isFinished() {
        // Ends the command if the sensor reads unknown or the current color matches
        return !m_armedRaised ||
            m_allianceColor == PPColor.Unknown ||
            calculateFieldSensorColor(m_currentColor) == m_allianceColor;
    }

    // This is the default command for the Power Panel Manipulator Subsystem
    private PPColor getStage3AllianceColor() {
        String gameData;
        PPColor allianceColor = PPColor.Unknown;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B':
                    allianceColor = PPColor.Blue;
                    break;
                case 'G':
                    allianceColor = PPColor.Green;
                    break;
                case 'R':
                    allianceColor = PPColor.Red;
                    break;
                case 'Y':
                    allianceColor = PPColor.Yellow;
                    break;
                default:
                    // Corrupted Data
                    allianceColor = PPColor.Unknown;
                    break;
            }
        } else {
            // No Data yet
            allianceColor = PPColor.Unknown;
        }

        return allianceColor;
    }

    private void updateDashboard() {
        SmartDashboard.putString("Current Color", m_currentColor.toString());
        SmartDashboard.putString("Calculated Color", calculateFieldSensorColor(m_currentColor).toString());
        SmartDashboard.putString("Alliance Color", m_allianceColor.toString());
    }

    public PPColor calculateFieldSensorColor(PPColor currentColor) {
        PPColor nextColor = PPColor.Unknown;

        if (currentColor == PPColor.Yellow) {
            nextColor = PPColor.Green;
        } else if (currentColor == PPColor.Blue) {
            nextColor = PPColor.Red;
        } else if (currentColor == PPColor.Green) {
            nextColor = PPColor.Yellow;
        } else if (currentColor == PPColor.Red) {
            nextColor = PPColor.Blue;
        }
        return nextColor;
    }

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
