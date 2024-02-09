// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwitchDrivetrainCommand;
import frc.robot.commands.ClimberInABoxCommand;
import frc.robot.commands.MoveIntakeCommand;
import frc.robot.commands.MoveIntakeWheelCommand;
import frc.robot.commands.OctocanumDrivetrainCommand;
import frc.robot.commands.ResetGryoCommand;
import frc.robot.commands.SetAmpShooterArmPositionCommand;
import frc.robot.commands.SetRobotAngleCommand;
import frc.robot.commands.SetShooterAngleCommand;
import frc.robot.commands.SetShooterSpeedCommand;
import frc.robot.commands.ShootNoteCommand;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.AngleShooter;
import frc.robot.subsystems.ClimberInAnBox;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

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
    xboxController = new CommandXboxController(0);
    xboxController1 = new CommandXboxController(1);
    drivetrain.setDefaultCommand(new OctocanumDrivetrainCommand(xboxController, drivetrain));
    climberInAnBox.setDefaultCommand(new ClimberInABoxCommand(climberInAnBox, xboxController1));
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

    xboxController.x().onTrue(new ResetGryoCommand(drivetrain));
    xboxController.y().whileTrue(new SetRobotAngleCommand(drivetrain, limelight));
    xboxController1
        .a()
        .whileTrue(
            new SequentialCommandGroup(new SetRobotAngleCommand(drivetrain, limelight),new ParallelCommandGroup(
                new SetShooterAngleCommand(angleShooter, limelight),
                new SetShooterSpeedCommand(shooter, 4000)
                ))
            );

    xboxController1
        .b()
        .whileTrue(
            new SequentialCommandGroup(
                new MoveIntakeCommand(intake, true), new MoveIntakeWheelCommand(intake, 1)))
        .onFalse(new MoveIntakeCommand(intake, false));

    xboxController1.leftBumper().whileTrue(new ShootNoteCommand(shooter, intake));
    xboxController1.povUp().onTrue(new SetAmpShooterArmPositionCommand(ampShooter, true));
    xboxController1.povDown().onTrue(new SetAmpShooterArmPositionCommand(ampShooter, false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("New Auto");
  }
}
