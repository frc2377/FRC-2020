package frc.robot.commands.autonomous;


import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.commandGroups.ShootPowerCellsCommand;
import frc.robot.commands.commandGroups.StopShootingCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.subsystems.*;

import java.io.IOException;
import java.nio.file.Path;

public class RightStartCommandGroup extends SequentialCommandGroup {
  public RightStartCommandGroup(DriveSubsystem drive,
                                TurretSubsystem turret,
                                CannonSubsystem cannon,
                                ConveyorSubsystem conveyor,
                                IntakeSubsystem intake) throws IOException {

    // Generate trajectories from Pathweaver json files
    // This test trajectory should drive in a circle
    //String trajectoryJSON = "paths/Right01Start.wpilib.json";
    String trajectoryJSON = "paths/Right01Start.wpilib.json";

    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    Trajectory testTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    Pose2d initialPosition = testTrajectory.getInitialPose();
    drive.resetOdometryPose(initialPosition, drive.getHeading());

    addCommands(
        new ShootPowerCellsCommand(cannon, conveyor, intake),
        new InstantCommand(() -> Timer.delay(3)),   
        new StopShootingCommand(cannon, conveyor, intake),
        new DeployIntakeCommand(intake),                      
        new DriveTrajectoryCommand(testTrajectory, drive),
        new RetractIntakeCommand(intake),
        new InstantCommand(() -> drive.tankDriveVolts(0, 0), drive)
    );

  }
}