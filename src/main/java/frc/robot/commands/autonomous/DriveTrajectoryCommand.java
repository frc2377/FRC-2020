/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTrajectoryCommand extends RamseteCommand {
    /**
     * Creates a new DriveTrajectory.
     */
    public DriveTrajectoryCommand(Trajectory trajectory, DriveSubsystem drive) {
        super(trajectory, drive::getPose, new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                drive.getFeedforward(),
                drive.getKinematics(), drive::getWheelSpeeds, new PIDController(DriveConstants.kKp, 0, 0),
                new PIDController(DriveConstants.kKp, 0, 0), drive::tankDriveVolts, drive);
    }
}
