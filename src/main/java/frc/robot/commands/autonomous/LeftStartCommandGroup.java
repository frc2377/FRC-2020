package frc.robot.commands.autonomous;


import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import java.io.IOException;
import java.nio.file.Path;

public class LeftStartCommandGroup extends SequentialCommandGroup {
  public LeftStartCommandGroup(DriveSubsystem drive,
                               TurretSubsystem turret,
                               CannonSubsystem cannon,
                               IntakeSubsystem intake) throws IOException {

    // Generate trajectories from Pathweaver json files
    // This test trajectory should drive in a circle
    //String trajectoryJSON = "paths/Right01Start.wpilib.json";
    String trajectoryJSON = "paths/Right01Start.wpilib.json"; // TODO: Use the correct json file(s) for left start

    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    Trajectory testTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    Pose2d initialPosition = testTrajectory.getInitialPose();
    drive.resetOdometryPose(initialPosition, drive.getHeading());

    addCommands(
        new DriveTrajectoryCommand(testTrajectory, drive),
        new InstantCommand(() -> drive.tankDriveVolts(0, 0), drive)
    );

  }
}