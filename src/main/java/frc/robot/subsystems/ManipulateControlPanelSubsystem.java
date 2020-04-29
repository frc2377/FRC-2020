/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ManipulateControlPanelConstants;

public class ManipulateControlPanelSubsystem extends SubsystemBase {
    private final WPI_TalonSRX m_motor;
    private final ColorSensorV3 m_colorSensor;
    private final ColorMatch m_colorMatcher;
    private final Solenoid m_solenoid;

    // We use an enum here to avoid typos while making comparisons from one color to
    // another and limit the set of our choices.
    public static enum PPColor {
        Blue, Red, Green, Yellow, Unknown
    }

    /**
     * Creates a new ManipulatePowerPanelSubsystem.
     */
    public ManipulateControlPanelSubsystem() {
        m_motor = new WPI_TalonSRX(ManipulateControlPanelConstants.kControlPanelMotorPort);
        m_motor.configFactoryDefault();
        m_motor.setNeutralMode(NeutralMode.Brake);
        m_motor.setInverted(ManipulateControlPanelConstants.kInvertMotor);
        m_motor.setSensorPhase(ManipulateControlPanelConstants.kInvertSensorDirection);
        m_motor.config_kF(Constants.kVelocityPIDSlotIdx0, ManipulateControlPanelConstants.kF, Constants.kTimeoutMs);
        m_motor.config_kP(Constants.kVelocityPIDSlotIdx0, ManipulateControlPanelConstants.kP, Constants.kTimeoutMs);
        m_motor.config_kI(Constants.kVelocityPIDSlotIdx0, ManipulateControlPanelConstants.kI, Constants.kTimeoutMs);
        m_motor.config_kD(Constants.kVelocityPIDSlotIdx0, ManipulateControlPanelConstants.kD, Constants.kTimeoutMs);
        m_motor.config_IntegralZone(Constants.kVelocityPIDSlotIdx0, ManipulateControlPanelConstants.kIzone);
        m_motor.configClosedloopRamp(ManipulateControlPanelConstants.kClosedLoopRamp, Constants.kTimeoutMs);
        m_motor.selectProfileSlot(Constants.kVelocityPIDSlotIdx0, Constants.kPidIdx0);

        m_colorSensor = new ColorSensorV3(ManipulateControlPanelConstants.kColorSensorPort);
        m_colorMatcher = new ColorMatch();
        m_solenoid = new Solenoid(ManipulateControlPanelConstants.kControlPanelSolenoidPort);


        m_colorMatcher.addColorMatch(ManipulateControlPanelConstants.kBlueTarget);
        m_colorMatcher.addColorMatch(ManipulateControlPanelConstants.kGreenTarget);
        m_colorMatcher.addColorMatch(ManipulateControlPanelConstants.kRedTarget);
        m_colorMatcher.addColorMatch(ManipulateControlPanelConstants.kYellowTarget);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Reads the sensor and tries to match it to one of the disk colors. Updates the
     * state of the current color and stores the previous color so color changes can
     * be detected.
     */
    public PPColor readColorSensor() {
        final Color detectedColor = m_colorSensor.getColor();
        // String colorString;
        final ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        PPColor ppColor = PPColor.Unknown;

        if (match.color == ManipulateControlPanelConstants.kBlueTarget) {
            ppColor = PPColor.Blue;
        } else if (match.color == ManipulateControlPanelConstants.kRedTarget) {
            ppColor = PPColor.Red;
        } else if (match.color == ManipulateControlPanelConstants.kGreenTarget) {
            ppColor = PPColor.Green;
        } else if (match.color == ManipulateControlPanelConstants.kYellowTarget) {
            ppColor = PPColor.Yellow;
        }

        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color", ppColor.toString());

        return ppColor;
    }

    /**
     * Get the Color currently being read by the color sensor, which contains the RGB value.
     * @return Color
     */
    public Color getDetectedColor() {
        return m_colorSensor.getColor();
    }

    public void displayRGBValue() {
        /**
         * The method GetColor() returns a normalized color value from the sensor and
         * can be useful if outputting the color to an RGB LED or similar. To read the
         * raw color, use GetRawColor().
         *
         * The color sensor works best when within a few inches from an object in well
         * lit conditions (the built in LED is a big help here!). The farther an object
         * is the more light from the surroundings will bleed into the measurements and
         * make it difficult to accurately determine its color.
         */

        final Color detectedColor = m_colorSensor.getColor();

        /**
         * The sensor returns a raw IR value of the infrared light detected.
         */
        final double IR = m_colorSensor.getIR();

        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the sensor.
         */
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);

        /**
         * In addition to RGB IR values, the color sensor can also return an infrared
         * proximity value. The chip contains an IR led which will emit IR pulses and
         * measure the intensity of the return. When an object is close the value of the
         * proximity will be large (max 2047 with default settings) and will approach
         * zero when the object is far away.
         *
         * Proximity can be used to roughly approximate the distance of an object or
         * provide a threshold for when an object is close enough to provide accurate
         * color values.
         */
        final int proximity = m_colorSensor.getProximity();

        SmartDashboard.putNumber("Proximity", proximity);
    }

    public boolean inRange() {
        final int proximity = m_colorSensor.getProximity();
        SmartDashboard.putNumber("Proximity", proximity);
        return proximity > ManipulateControlPanelConstants.kRangeThreshold;
    }

    public void rotatePowerPanelMotor(double speed) {
        // The logic for how to rotate the panel goes in the command, so this just says
        // to rotate the motor. Figure out when to stop in the corresponding command.
        // Rotates motor at speed
        m_motor.set(ControlMode.PercentOutput, speed);
    }
    public void stopPowerPanelMotor() {
        // Stops the power panel motor.
        m_motor.stopMotor();
    }

    public boolean controlPanelArmDeployed() {
        return m_solenoid.get();
    }


    public void log() {

    }

    /**
     * Toggle the power panel color sensor arm
     */
    public void togglePowerPanelArm() {
        m_solenoid.set(!m_solenoid.get());
    }

    public void brakeMotor(){
        m_motor.setNeutralMode(NeutralMode.Brake);
    }
    public void releaseMotor(){
        m_motor.setNeutralMode(NeutralMode.Coast);
    }
}
