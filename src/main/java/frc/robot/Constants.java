/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  // The period is 20ms for RoboRio
  public static final double kDt = 0.02;
  public static final int kVelocityPIDSlotIdx0 = 0; // Inner Loop
  public static final int kPositionPIDSlotIdx1 = 1; // Talon slot 1
  public static final int kPidIdx0 = 0; // Primary PID Loop 0
  public static final int kTimeoutMs = 10;


  // Constants for the Drive Subsystem
  public static final class DriveConstants {
    // Free speed of Falcon 500 per documentation is 6380 RPM
    // Calculated kV = (12V / (6380 * .1016 * Math.PI)) / 10.52 / 60 = 3.7V
    public static final int kLeftFrontMotorPort = 15;
    public static final int kLeftRearMotorPort = 14;
    public static final int kRightFrontMotorPort = 13;
    public static final int kRightRearMotorPort = 12;

    public static final boolean kLeftInvertMotor = false;
    public static final boolean kLeftInvertSensorPhase = true;

    public static final boolean kRightInvertMotor = true;
    public static final boolean kRightInvertSensorPhase = true;

    public static final double kTrackwidthMeters = 0.65;
    public static final double kWheelDiameter = (Units.inchesToMeters(4.58));
    public static final double kEncoderPulsesPerRevolution = 2048;
    public static final double kDriveGearRatio = 10.52;
    public static final double ksVolts = 0.0732;
    public static final double kvVoltSecondsPerMeter = 3.28;
    public static final double kaVoltSecondsSquaredPerMeter = 0.292;

    public static final boolean kGyroReversed = true;

    public static final double kKp = 0.00182;

    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;

    public static final double kDistancePerRevolution = (kWheelDiameter * Math.PI)
        / (kDriveGearRatio * kEncoderPulsesPerRevolution);

    public static final double kSpeedMultiplier = 0.7;

  }

  // Constants for the Operator Interface (OI)
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 1;
    public static final int kTestControllerPort = 2;
  }

  // Constants for Autonomous Mode
  public static final class AutoConstants {
    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  // Constants for the Intake Subsystem

  public static final class IntakeConstants {
    public static final int kIntakeMotor = 11;
    public static final boolean kInvertMotor = false;
    public static final boolean kInvertSensorDirection = false;
    public static final int kIzone = 250;
    public static final double kClosedLoopRamp = 0.02;
    public static final double kMaxMotorVelocity = 91256;

    // TODO: Set PID for velocity
    public static final double kP = 0.015;
    public static final double kI = 0.0001;
    public static final double kD = 0.4;
    public static final double kF = 0.011;

    public static final int kPPR = 4096;

    public static final int kIntakeSolenoid = 3;
    public static final double kMotorSpeed = 1.0;
  }

  public static final class ConveyorConstants {
    public final static int kConveyorMotor = 5;
    public final static double kConveyorMotorVelocity = 7418;
    public static final boolean kInvertMotor = false;
    public static final boolean kInvertSensorDirection = true;
    public static final double kClosedLoopRamp = 0.02;
    public static final int kIzone = 250;

    // Velocity PID
    public static final double kF_vel = 0.128;
    public static final double kP_vel = 0.1;
    public static final double kI_vel = 0.001;
    public static final double kD_vel = 2;

    // Position PID
    public static final double kP_pos = 3;
    public static final double kI_pos = 0;
    public static final double kD_pos = 40;

    public static final int kAdvanceOnePowerCellDistance = 3000;
    public static final int kVelocityTolerance = 1000;
  }

  public static final class MagazineConstants {
    public static final int kGarageSensorPort1 = 4; // Front of Droid
    public static final int kGarageSensorPort2 = 3;
    public static final int kGarageSensorPort3 = 2;
    public static final int kGarageSensorPort4 = 1;
    public static final int kGarageSensorPort5 = 0; // Nearest to Hop Up Motor
  }

  // Constants for the Climb Subsystem
  public static final class ClimbConstants {
    // Front
    public static final int kFrontClimbMotorPort = 2;
    public static final boolean kFrontInvertMotor = false;
    public static final boolean kFrontInvertSensorDirection = true;
    public static final int kFrontIzone = 100;
    public static final double kP_front = 0.15; // Tune value for PID
    public static final double kI_front = 0; // Tune value for PID
    public static final double kD_front = 7; // Tune value for PID
    public static final double kF_front = 0.128; // Tune value for PID

    // Rear
    public static final int kRearClimbMotorPort = 3;
    public static final boolean kRearInvertMotor = true;
    public static final boolean kRearInvertSensorDirection = false;
    public static final int kRearIzone = 100;
    public static final double kP_rear = 0.11; // Tune value for PID
    public static final double kI_rear = 0; // Tune value for PID
    public static final double kD_rear = 2.787; // Tune value for PID
    public static final double kF_rear = 0.077; // Tune value for PID

    // Buddy
    public static final int kBuddyClimbMotorPort = 0; // TODO: This isn't correct.  Verify port number
    public static final boolean kBuddyInvertSensorDirection = false;
    public static final boolean kBuddyInvertMotor = false;
    public static final int kBuddyIzone = 100;
    public static final int kP_buddy = 0; // Tune value for PID
    public static final int kI_buddy = 0; // Tune value for PID
    public static final int kD_buddy = 0; // Tune value for PID

    public static final double kClosedLoopRamp = 0.02;


    public static final int kClimberSolenoid = 1;

    public static final int kBuddySolenoidPCMChannel = 0;

    public static final int kFrontForwardSoftLimit = 2000;
    public static final int kFrontReverseSoftLimit = 0;
    public static final int kRearForwardSoftLimit = 63000;
    public static final int kRearReverseSoftLimit = 0;

    public static final double kRearMotorSpeed = 1000;
    public static final double kFrontMotorSpeed = 1000;

    public static final double kRearDeploySpeed = 10000;
    public static final double kFrontDeploySpeed = kRearDeploySpeed / 2;
    public static final double kTiltThreshold = 3;
  }

  // Constants for the Power Panel Manipulator
  public static final class ManipulateControlPanelConstants {
    public static final int kControlPanelMotorPort = 7;
    public static final boolean kInvertSensorDirection = true;
    public static final boolean kInvertMotor = true;
    public static final int kIzone = 250;
    public static final double kClosedLoopRamp = 0.02;

    // TODO: Tune the PID
    public static final int kF = 0;
    public static final int kP = 0;
    public static final int kI = 0;
    public static final int kD = 0;

    public static final int kControlPanelSolenoidPort = 2;

    public static final I2C.Port kColorSensorPort = I2C.Port.kOnboard;
    public static final Color kBlueTarget = ColorMatch.makeColor(0.193, 0.445, 0.360);
    public static final Color kGreenTarget = ColorMatch.makeColor(0.217, 0.533, 0.249);
    public static final Color kRedTarget = ColorMatch.makeColor(0.420, 0.397, 0.181);
    public static final Color kYellowTarget = ColorMatch.makeColor(0.324, 0.538, 0.136);

    // TODO: Tune the range threshold getProximity() returns a value between 0 (far) and 2047 (close)
    public static final int kRangeThreshold = 1000;

    public static int kStage2ColorCount = 28; // To rotate 3.5 times, count 28 color changes
    public static double kControlPanelMotorVelocity = 0.3; // TODO: Set velocity
  }

  // Constants for the Turret
  public static final class TurretConstants {
    public static final int kTurretMotorPort = 6;

    public static final boolean kInvertSensorDirection = false;
    public static final boolean kInvertMotor = false;
    public static final int kIzone = 200;
    public static final double kClosedLoopRamp = 0.02;

    public static final double kF = 0.0;
    public static final double kP = 1.5;
    public static final double kI = 0;
    public static final double kD = 25;

    private static final double kSoftLimitDegrees = 20.0;
    public static final double kGearRatio = 155.0 / 12;
    public static final int kPPR = 4096;
    public static final int kSoftLimit = (int) (kGearRatio * kPPR * kSoftLimitDegrees) / 360;

    public static final double kArbitraryFeedForward = 0.1;
  }

  // Constants for the Cannon
  public static final class CannonConstants {
    // Lower Cannon Motor
    public static final int kLowerCannonMotorPort = 1;
    public static final boolean kLowerInvertMotor = false;
    public static final boolean kLowerInvertSensorDirection = true;
    public static final int kLowerIzone = 100;
    public static final double kLowerMaxMotorVelocity = 54264;
    // TODO: set lower PID to correct values
    // Upper PID values
    public static final double kF_lower = 0.018;
    public static final double kP_lower = 0.05;
    public static final double kI_lower = 0;
    public static final double kD_lower = 3;

    // Upper Cannon motor
    public static final int kUpperCannonMotorPort = 20;
    public static final boolean kUpperInvertMotor = true;
    public static final boolean kUpperInvertSensorDirection = true;
    public static final int kUpperIzone = 100;
    public static final double kUpperMaxMotorVelocity = 36648;
    // TODO: set upper PID to correct values
    // Upper PID values
    public static final double kF_upper = 0.027;
    public static final double kP_upper = 0.05;
    public static final double kI_upper = 0;
    public static final double kD_upper = 2;

    // Hopup motor
    public static final int kHopupMotorPort = 4;
    public static final boolean kHopupInvertMotor = false;
    public static final boolean kHopupInvertSensorDirection = true;
    public static final int kHopupIzone = 250;
    public static final double kHopupMaxMotorVelocity = 8242;

    public static final double kF_hopup = 0.128;
    public static final double kP_hopup = 0.15;
    public static final double kI_hopup = 0;
    public static final double kD_hopup = 3.4;
    public static final double kHopupMotorVelocity = 7650;

    public static final double kClosedLoopRamp = 0.02;

    public static final int kLowerMaxSpeed = 54200;
    public static final int kUpperMaxSpeed = 36700;

    // these values are NOT real or important
    // TODO: set real setpoints
    public static final int[][] kTacticalTargetingTable = new int[][]{
        {(int)(kLowerMaxSpeed * 1.0), (int)(kUpperMaxSpeed * 1.0)},
        {(int)(kLowerMaxSpeed * 0.8), (int)(kUpperMaxSpeed * 0.65)},
        {(int)(kLowerMaxSpeed * 0.65), (int)(kUpperMaxSpeed * 0.60)},
        {(int)(kLowerMaxSpeed * 0.65), (int)(kUpperMaxSpeed * 0.65)},
        {(int)(kLowerMaxSpeed * 0.65), (int)(kUpperMaxSpeed * 0.75)},
        {(int)(kLowerMaxSpeed * 0.70), (int)(kUpperMaxSpeed * 0.72)}
    };

    public static final int kPPR = 4096;
    public static final int kVelocityTolerance = 1000;

  }

  public static final class LedConstants {
    public static final int kPWMPort = 1;
    public static final int kLengthFfLedChain = 16;
    public static final int kTimerLength = 7;
  }

  public static final class LimelightConstants {
    // Refer to page 31 of the limelight documentation release 1.0.
    public static final double kH2 = 2.877; // 113.25 inches
    public static final double kH1 = 0.406; // 16 inches
    public static final double kA1 = 45;
  }
}
