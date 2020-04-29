/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// Programmed by Logan Cuttler

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;

public class LedSubsystem extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private boolean policeLightsActive;
    private int timer;

    /**
     * Creates a new LedSubsystem.
     */
    public LedSubsystem() {
        m_led = new AddressableLED(LedConstants.kPWMPort);
        m_ledBuffer = new AddressableLEDBuffer(LedConstants.kLengthFfLedChain); // 33 or 16
        m_led.setLength(m_ledBuffer.getLength());

        policeLightsActive = false;
        timer = 0;

        m_led.start();
    }

    /**
     * Sets the led to the given RGB values.
     *
     * @param r red int
     * @param g green int
     * @param b blue int
     */
    public void displayColorOnLED(int r, int g, int b) { // Based on the color that the colorsensor sees, we change the leds
        // to that
        // color
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    public void displayColorOnLED(double r, double g, double b) { // Based on the color that the colorsensor sees, we change
        // the leds to that color
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, (int) (r * 255), (int) (g * 255), (int) (b * 255));
        }
        m_led.setData(m_ledBuffer);
    }

    public void changeColorBlue() {

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 255);
        }
        m_led.setData(m_ledBuffer);
    }

    public void changeColorGreen() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 255, 0);
        }
        m_led.setData(m_ledBuffer);

    }

    public void togglePoliceLights() {
        if (timer == LedConstants.kTimerLength) {
            if (policeLightsActive) {
                for (var i = 1; i < m_ledBuffer.getLength() - 1; i++) { // Red Light with flashing ends
                    m_ledBuffer.setRGB(0, 255, 255, 255);
                    m_ledBuffer.setRGB(m_ledBuffer.getLength() - 1, 0, 0, 0);
                    m_ledBuffer.setRGB(i, 255, 0, 0);
                }
                timer = 0;
            } else {
                for (var i = 1; i < m_ledBuffer.getLength() - 1; i++) { // Blue light with flashing ends
                    m_ledBuffer.setRGB(0, 0, 0, 0);
                    m_ledBuffer.setRGB(m_ledBuffer.getLength() - 1, 255, 255, 255);
                    m_ledBuffer.setRGB(i, 0, 0, 255);
                }
                timer = 0;
            }
            policeLightsActive = !policeLightsActive;
        }
        m_led.setData(m_ledBuffer);
        timer++;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
