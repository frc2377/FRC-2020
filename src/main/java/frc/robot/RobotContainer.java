/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Triggers.XboxTrigger;
import frc.robot.commands.autonomous.CenterStartCommandGroup;
import frc.robot.commands.autonomous.LeftStartCommandGroup;
import frc.robot.commands.autonomous.RightStartCommandGroup;
import frc.robot.commands.climb.ClimbUpCommand;
import frc.robot.commands.climb.DeployArmsCommand;
import frc.robot.commands.climb.RetractFrontMotorCommand;
import frc.robot.commands.climb.RetractRearMotorCommand;
import frc.robot.commands.commandGroups.ShootPowerCellsCommand;
import frc.robot.commands.commandGroups.StopShootingCommand;
import frc.robot.commands.conveyor.ConveyorSubsystemDefaultCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.commands.manipulatecontrolpanel.AutoRotateStage2Command;
import frc.robot.commands.manipulatecontrolpanel.AutoRotateStage3Command;
import frc.robot.commands.turret.AlignToTargetCommand;
import frc.robot.subsystems.*;
import frc.robot.utils.Limelight;
import frc.robot.utils.Utils;

import java.io.IOException;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //   Define subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ConveyorSubsystem m_conveyor = new ConveyorSubsystem();
  private final ClimbSubsystem m_climb = new ClimbSubsystem();
  private final CannonSubsystem m_cannon = new CannonSubsystem();
  private final ManipulateControlPanelSubsystem m_powerPanel = new ManipulateControlPanelSubsystem();
  private final TurretSubsystem m_turret = new TurretSubsystem();
//  private final LedSubsystem m_ledSubsystem = new LedSubsystem();

  // Define Driver controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  // Define Manipulator controller
  XboxController m_manipulatorController = new XboxController(OIConstants.kManipulatorControllerPort);

  // Define a controller for testing functions
  XboxController m_testController = new XboxController(OIConstants.kTestControllerPort);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
//    configureTestButtons();

    // Configure default commands
//    m_drive.setDefaultCommand(
//        new ArcadeDriveCommand(() -> m_driverController.getY(GenericHID.Hand.kLeft) * Constants.DriveConstants.kSpeedMultiplier * -1,
//        () -> m_driverController.getX(GenericHID.Hand.kRight) * Constants.DriveConstants.kSpeedMultiplier, m_drive));
    m_conveyor.setDefaultCommand(new ConveyorSubsystemDefaultCommand(m_conveyor));
    m_turret.setDefaultCommand(new RunCommand(
        () -> m_turret.runMotor(m_manipulatorController.getX(GenericHID.Hand.kRight), 3), m_turret)
    );
//    m_turret.setDefaultCommand(new AlignToTargetCommand(m_turret));


    // Show what command our subsystems are running on the SmartDashboard
    SmartDashboard.putData(m_drive);
    SmartDashboard.putData(m_intake);
    SmartDashboard.putData(m_conveyor);
    SmartDashboard.putData(m_cannon);
    SmartDashboard.putData(m_climb);
    SmartDashboard.putData(m_powerPanel);
    SmartDashboard.putData(m_turret);

    // Configure autonomous commands for chooser
    Command rightStartCommand;
    try {
      rightStartCommand = new RightStartCommandGroup(m_drive, m_turret, m_cannon, m_conveyor, m_intake);
    } catch (IOException ex) {
      DriverStation.reportError("Unable load right start autonomous command", ex.getStackTrace());
      rightStartCommand = new InstantCommand();
    }

    Command centerStartCommand;
    try {
      centerStartCommand = new CenterStartCommandGroup(m_drive, m_turret, m_cannon, m_intake);
    } catch (IOException ex) {
      DriverStation.reportError("Unable load center start autonomous command", ex.getStackTrace());
      centerStartCommand = new InstantCommand();
    }

    Command leftStartCommand;
    try {
      leftStartCommand = new LeftStartCommandGroup(m_drive, m_turret, m_cannon, m_intake);
    } catch (IOException ex) {
      DriverStation.reportError("Unable load left start autonomous command", ex.getStackTrace());
      leftStartCommand = new InstantCommand();
    }


    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Right Auto", rightStartCommand);
    // m_chooser.setDefaultOption("Right Auto", new PrintCommand("Chose right auto command"));
    m_chooser.addOption("Center Auto", centerStartCommand);
    // m_chooser.addOption("Center Auto", new PrintCommand("Chose center auto command"));
    m_chooser.addOption("Left Auto", leftStartCommand);
    // m_chooser.addOption("Left Auto", new PrintCommand("Chose left auto command"));

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);

    Limelight.disableLED();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {


    /* ******************************************
     * Driver button mappings
     ********************************************/

    /* Map IntakeSubsystem Commands to Driver Controller */

    // Deploy Intake
    new JoystickButton(m_driverController, Button.kBumperRight.value)
        .whileHeld(new DeployIntakeCommand(m_intake))
        .whenReleased(new RetractIntakeCommand(m_intake));


    new JoystickButton(m_driverController, Button.kB.value)
        .whenPressed(new InstantCommand(() -> m_turret.turnToPosition(Utils.degrees2Tics(5, Constants.TurretConstants.kGearRatio, Constants.TurretConstants.kPPR)), m_turret));

    new JoystickButton(m_driverController, Button.kX.value)
        .whenPressed(new InstantCommand(() -> m_turret.turnToPosition(Utils.degrees2Tics(-5, Constants.TurretConstants.kGearRatio, Constants.TurretConstants.kPPR)), m_turret));

    new JoystickButton(m_driverController, Button.kY.value)
        .whenPressed(new InstantCommand(() -> m_turret.turnToPosition(Utils.degrees2Tics(0, Constants.TurretConstants.kGearRatio, Constants.TurretConstants.kPPR)), m_turret));



    /* ********************************************
     * Manipulator button mappings
     **********************************************/

    // TODO: This binding is experimental
    // Enable auto tracking while holding the left trigger
    new XboxTrigger(m_manipulatorController, GenericHID.Hand.kLeft, 0.1)
        .whileActiveContinuous(new AlignToTargetCommand(m_turret));

    // TODO: This binding is experimental
    // Fire the cannon if the Limelight has a target while holding down the right trigger
    new XboxTrigger(m_manipulatorController, GenericHID.Hand.kRight, 0.1)
        .whenActive(new ShootPowerCellsCommand(m_cannon, m_conveyor, m_intake))
        .whenInactive(new StopShootingCommand(m_cannon, m_conveyor, m_intake));

    // Press Y to toggle the control arm
    new JoystickButton(m_manipulatorController, Button.kY.value)
        .whenPressed(new InstantCommand(m_powerPanel::togglePowerPanelArm));


    // Assign Stage 2 and Stage 3 Power Panel buttons left bumper = stage 2, right bumper = stage 3
    new JoystickButton(m_manipulatorController, Button.kBumperLeft.value)
        .whenPressed(new AutoRotateStage2Command(m_powerPanel));
    new JoystickButton(m_manipulatorController, Button.kBumperRight.value)
        .whenPressed(new AutoRotateStage3Command(m_powerPanel));

    // Assign climb
    // Deploy
    new POVButton(m_manipulatorController, 270)
        .whileHeld(new RetractRearMotorCommand(m_climb));
    new POVButton(m_manipulatorController, 90)
        .whileHeld(new RetractFrontMotorCommand(m_climb));
    new POVButton(m_manipulatorController, 0)
        .whileHeld(new ClimbUpCommand(m_climb));
    new POVButton(m_manipulatorController, 180)
        .whileHeld(new DeployArmsCommand(m_climb));
  }

  private void configureTestButtons() {
    new JoystickButton(m_testController, Button.kA.value)
        .whenPressed(new InstantCommand(m_climb::disengageClimberSolenoid, m_climb))
        .whenReleased(new InstantCommand(m_climb::engageClimberSolenoid, m_climb));

    new JoystickButton(m_testController, Button.kB.value)
        .whenPressed(new InstantCommand(m_intake::deployIntake, m_intake))
        .whenReleased(new InstantCommand(m_intake::retractIntake, m_intake));

    new JoystickButton(m_testController, Button.kY.value)
        .whenPressed(new InstantCommand(m_powerPanel::togglePowerPanelArm, m_powerPanel));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    /* ***************************************************************************
     * We can do our decision making here and return multiple Auto commands/groups
     * based on some variable such as starting position.
     ****************************************************************************/
//    return m_chooser.getSelected();
    return new InstantCommand(); // TODO: Remove this and restore autonomous
  }
}
