// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpShooter;

public class SetAmpShooterSpeedCommand extends Command {
  private AmpShooter ampShooter;
  private boolean up;
  private boolean down;

  /** Creates a new SetAmpShooterSpeedCommand. */
  public SetAmpShooterSpeedCommand(boolean up, boolean down, AmpShooter ampShooter) {
    this.up = up;
    this.down = down;
    this.ampShooter = ampShooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ampShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (up) {
      ampShooter.setWheelPower(1);
    } else if (down) {
      ampShooter.setWheelPower(-1);
    } else {
      ampShooter.setWheelPower(0);
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
