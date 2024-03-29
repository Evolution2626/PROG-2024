// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.time.LocalTime;

public class WaitXSecondCommand extends Command {
  private long seconde;
  private LocalTime startTime;

  /** Creates a new ClimberInABoxCommand. */
  public WaitXSecondCommand(long seconde) {
    this.seconde = seconde;
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = LocalTime.now();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (startTime.plusSeconds(seconde).compareTo(LocalTime.now()) <= 0) { // check > or <
      return true;
    }
    return false;
  }
}
