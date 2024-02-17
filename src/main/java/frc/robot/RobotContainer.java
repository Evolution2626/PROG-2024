// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoShootToSideCommand;
import frc.robot.commands.ClimberInABoxCommand;
import frc.robot.commands.MoveIntakeCommand;
import frc.robot.commands.MoveIntakeWheelCommand;
import frc.robot.commands.OctocanumDrivetrainCommand;
import frc.robot.commands.SetAmpShooterArmPositionCommand;
import frc.robot.commands.SetAmpShooterSpeedCommand;
import frc.robot.commands.SetRobotAngleCommand;
import frc.robot.commands.SetShooterAngleCommand;
import frc.robot.commands.SetShooterSpeedCommand;
import frc.robot.commands.ShootNoteCommand;
import frc.robot.commands.StopShooterCommand;
import frc.robot.commands.SwitchDrivetrainCommand;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.AngleShooter;
import frc.robot.subsystems.ClimberInAnBox;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPusher;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private Drivetrain drivetrain;
  private ClimberInAnBox climberInAnBox;
  private Shooter shooter;
  private Intake intake;
  private AngleShooter angleShooter;
  private Limelight limelight;
  private AmpShooter ampShooter;
  private ShooterPusher shooterPusher;
  private CommandXboxController xboxController = new CommandXboxController(0);
  private CommandXboxController xboxController1 = new CommandXboxController(1);

  public RobotContainer() {
    // Configure the trigger bindings

    drivetrain = new Drivetrain();
    climberInAnBox = new ClimberInAnBox();
    intake = new Intake();
    shooter = new Shooter();
    limelight = new Limelight();
    angleShooter = new AngleShooter();
    ampShooter = new AmpShooter();
    shooterPusher = new ShooterPusher();
    xboxController = new CommandXboxController(0);
    xboxController1 = new CommandXboxController(1);
    drivetrain.setDefaultCommand(new OctocanumDrivetrainCommand(xboxController, drivetrain));
    climberInAnBox.setDefaultCommand(new ClimberInABoxCommand(climberInAnBox, xboxController1));
    limelight.setDefaultCommand(new SetShooterAngleCommand(angleShooter, limelight));
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    xboxController.a().onTrue(new SwitchDrivetrainCommand(drivetrain));
    xboxController.y().whileTrue(new SetRobotAngleCommand(drivetrain, limelight));
    xboxController1
        .a()
        .whileTrue(new SetShooterSpeedCommand(shooter))
        .onFalse(new StopShooterCommand(shooter));
        
    xboxController1
        .b()
        .whileTrue(
            new SequentialCommandGroup(
                new MoveIntakeCommand(intake, true), new MoveIntakeWheelCommand(intake, 1)))
        .onFalse(new MoveIntakeCommand(intake, false));

    xboxController1.leftBumper().whileTrue(new ShootNoteCommand(shooterPusher, intake));
    xboxController1.y().onTrue(new SetAmpShooterArmPositionCommand(ampShooter));
    xboxController1
        .povUp()
        .whileTrue(new SetAmpShooterSpeedCommand(true, false, ampShooter))
        .onFalse(new SetAmpShooterSpeedCommand(false, false, ampShooter));
    xboxController1
        .povUp()
        .whileTrue(new SetAmpShooterSpeedCommand(false, true, ampShooter))
        .onFalse(new SetAmpShooterSpeedCommand(false, false, ampShooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    // return autoChooser.getSelected();
    return new AutoShootToSideCommand(
        drivetrain, limelight, angleShooter, shooter, intake, shooterPusher);
    // Reset odometry to the initial pose of the trajectory, run path following
    // command, then stop at the end.

  }
}
