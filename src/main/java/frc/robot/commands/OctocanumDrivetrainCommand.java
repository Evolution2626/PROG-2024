// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.util.Range;

public class OctocanumDrivetrainCommand extends Command {
  private CommandXboxController xboxController;

  private Drivetrain drivetrain;

  /** Creates a new PistonOctocanum. */
  public OctocanumDrivetrainCommand(CommandXboxController xboxController, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.xboxController = xboxController;
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(
        Range.threshold(0.1, xboxController.getRightX()),
        Range.threshold(0.1, xboxController.getRightY()),
        Range.threshold(0.1, xboxController.getLeftX()),
        Range.threshold(0.1, xboxController.getLeftY()),
        Range.threshold(0.1, xboxController.getRightTriggerAxis()),
        Range.threshold(0.1, xboxController.getLeftTriggerAxis()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
