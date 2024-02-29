// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ShootNoteCommand extends Command {
  private Intake intake;

  /** Creates a new ShootNoteCommand. */
  public ShootNoteCommand(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   intake.resetWheelEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.spinWheel(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    intake.spinWheel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(intake.getWheelEncoder()) >= 20){//validate if positive or negative
      return true;
    }
    return false;
  }
}
