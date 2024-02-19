// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class MoveIntakeCommand extends Command {
  private Intake intake;
  boolean intakeOut;

  /** Creates a new MoveIntakeCommand. */
  public MoveIntakeCommand(Intake intake, boolean intakeOut) {
    this.intake = intake;
    this.intakeOut = intakeOut;

    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.getEncoder().getPosition() == 0) {
      intake.moveIntake(1);
    } else {
      intake.moveIntake(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.moveIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
