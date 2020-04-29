/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LimelightConstants;

/**
 * Utility class to provide easy access to retrieve and manipulate Limelight
 * NetworkTable entries.
 */
public final class Limelight {
    private static final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");
    private static final NetworkTableEntry m_tx = m_table.getEntry("tx");
    private static final NetworkTableEntry m_ty = m_table.getEntry("ty");
    private static final NetworkTableEntry m_ta = m_table.getEntry("ta");
    private static final NetworkTableEntry m_tv = m_table.getEntry("tv");
    private static final NetworkTableEntry m_ledMode = m_table.getEntry("ledMode");

    /**
     * Get the horizontal offset from the Limelight
     *
     * @return tx from the Limelight NetworkTable
     */
    public static double getTx() {
        return m_tx.getDouble(0.0);
    }

    /**
     * Get the vertical offset from the Limelight
     *
     * @return ty from the Limelight NetworkTable
     */
    public static double getTy() {
        return m_ty.getDouble(0.0);
    }

    /**
     * Get the target area from the limelight
     *
     * @return ta from the Limelight NetworkTable
     */
    public static double getTa() {
        return m_ta.getDouble(0.0);
    }

    /**
     * Get the target verification from the Limelight
     *
     * @return tv from the Limelight NetworkTable
     */
    public static double getTv() {
        return m_tv.getDouble(0.0);
    }

    /**
     * Check to see if Limelight has a target
     * 
     * @return True if Limelight has a target
     */
    public static boolean hasTarget() {
        return getTv() == 1.0;
    }
    
    /**
     * Get the distance to the target.  Returns 0.0 if no target available.
     * 
     * @return The distance to the target in meters
     */
    public static double getDistance() {
        double distance = 0.0;

        if (hasTarget()) {
            // Calculate the distance to the target

            double sumOfAngles = LimelightConstants.kA1 + getTy();
            double denominator = Math.tan(sumOfAngles);
            double numerator = LimelightConstants.kH2 - LimelightConstants.kH1;
            if (denominator != 0){
                distance = numerator / denominator;
            }
        }   
        return distance;
    }
    
    /**
     * Send current limelight NetworkTable entries to the SmartDashboard
     */
    public void log() {
        SmartDashboard.putNumber("Limelight X", getTx());
        SmartDashboard.putNumber("Limelight Y", getTy());
        SmartDashboard.putNumber("Limelight Area", getTa());
        SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
        SmartDashboard.putNumber("Distance to Target", getDistance());
    }

    public static void disableLED() {
        m_ledMode.setNumber(1);
    }

    public static void enableLED() {
        m_ledMode.setNumber(0);
    }
}
