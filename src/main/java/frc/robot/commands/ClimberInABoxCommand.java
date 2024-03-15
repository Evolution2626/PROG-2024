// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimberInAnBox;
import frc.util.Range;

public class ClimberInABoxCommand extends Command {
  private ClimberInAnBox climberInAnBox;
  private CommandXboxController xboxController;

  /** Creates a new ClimberInABoxCommand. */
  public ClimberInABoxCommand(ClimberInAnBox climberInAbox, CommandXboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberInAnBox = climberInAbox;
    this.xboxController = xboxController;
    addRequirements(climberInAbox);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*  if (Range.threshold(0.1, xboxController.getLeftY()) != 0
        || Range.threshold(0.1, xboxController.getRightY()) != 0) {
      if (xboxController.leftBumper().getAsBoolean()
          && xboxController.rightBumper().getAsBoolean()
          && climberInAnBox.getclimberOut()) {
        climberInAnBox.activateRatchet();
        climberInAnBox.climb(
            Math.abs(Range.threshold(0.1, xboxController.getLeftY())),
            Range.threshold(0.1, Math.abs(xboxController.getRightY())));
      } else if (!climberInAnBox.isRatchetActivated()) {
        climberInAnBox.setClimberOut(true);
        climberInAnBox.climb(
            -Math.abs(Range.threshold(0.1, xboxController.getLeftY())),
            -Math.abs(Range.threshold(0.1, xboxController.getRightY()))); // TODO check orientation
      }
    } else {
      climberInAnBox.climb(0, 0);
    }*/
    if (xboxController.rightBumper().getAsBoolean() && xboxController.leftBumper().getAsBoolean()) {
      climberInAnBox.disactivateRatchet();
      climberInAnBox.climb(-xboxController.getLeftY(), -xboxController.getRightY());
    } else {
      climberInAnBox.activateRatchet();
      climberInAnBox.climb(
          Range.coerce(-1, 0, Range.threshold(0.1, -xboxController.getLeftY())),
          Range.coerce(-1, 0, Range.threshold(0.1, -xboxController.getRightY())));
    }
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
