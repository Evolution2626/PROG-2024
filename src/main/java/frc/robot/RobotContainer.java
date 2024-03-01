// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoShootToSideCommand;
import frc.robot.commands.ClimberInABoxCommand;
import frc.robot.commands.GetAwayAutoCommand;
import frc.robot.commands.MoveIntakeCommand;
import frc.robot.commands.OctocanumDrivetrainCommand;
import frc.robot.commands.SetAmpStateCommand;
import frc.robot.commands.SetIntakeStateCommand;
import frc.robot.commands.SetRobotAngleCommand;
import frc.robot.commands.SetShooterAngleCommand;
import frc.robot.commands.SetShooterSpeedCommand;
import frc.robot.commands.SetShooterStateCommand;
import frc.robot.commands.SwitchDrivetrainCommand;
import frc.robot.subsystems.Amp;
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
  private Amp amp;
 // private AmpShooter ampShooter;
  private CommandXboxController xboxController = new CommandXboxController(0);
  private CommandXboxController xboxController1 = new CommandXboxController(1);

  private  Command m_avancer;
  private  Command m_shoot;
      SendableChooser<Command> m_chooser = new SendableChooser<>();
      

  public RobotContainer() {
    // Configure the trigger bindings

    drivetrain = new Drivetrain();
    climberInAnBox = new ClimberInAnBox();
    intake = new Intake();
    shooter = new Shooter();
    limelight = new Limelight();
    angleShooter = new AngleShooter();
    amp = new Amp();
    //ampShooter = new AmpShooter();
    drivetrain.setDefaultCommand(new OctocanumDrivetrainCommand(xboxController, drivetrain));
    climberInAnBox.setDefaultCommand(new ClimberInABoxCommand(climberInAnBox, xboxController1));
    intake.setDefaultCommand(new MoveIntakeCommand(intake, xboxController1));
    limelight.setDefaultCommand(new SetShooterAngleCommand(limelight, angleShooter));
    shooter.setDefaultCommand(new SetShooterSpeedCommand(shooter, amp));
    configureBindings();
    m_chooser.setDefaultOption("avancer", m_avancer);
    m_chooser.addOption("shooter", m_shoot);
    SmartDashboard.putData(m_chooser);
    SmartDashboard.putNumber("Auto Wait Time", 0);
    m_avancer =
      new GetAwayAutoCommand(drivetrain);
     m_shoot =
      new AutoShootToSideCommand(drivetrain, limelight, angleShooter, shooter, intake, amp);
     
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * 
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    xboxController.a().onTrue(new SwitchDrivetrainCommand(drivetrain));
    xboxController.b().whileTrue(new SetRobotAngleCommand(drivetrain, limelight));

    xboxController1.b().onTrue(new SetIntakeStateCommand(intake));
    xboxController1.a().onTrue(new SetShooterStateCommand(shooter));
    xboxController1.x().onTrue(new SetAmpStateCommand(shooter));
    xboxController.y().onTrue(new SetRobotAngleCommand(drivetrain, limelight));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    // return autoChooser.getSelected();
    //return m_chooser.getSelected();
    return new AutoShootToSideCommand(drivetrain, limelight, angleShooter, shooter, intake, amp);
    // Reset odometry to the initial pose of the trajectory, run path following
    // command, then stop at the end.

  }
}
