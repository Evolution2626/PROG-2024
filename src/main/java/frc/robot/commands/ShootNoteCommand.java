// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPusher;

public class ShootNoteCommand extends Command {
  private ShooterPusher shooter;
  private Intake intake;
  private double initPos;
  /** Creates a new ShootNoteCommand. */
  public ShootNoteCommand(ShooterPusher shooter, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.intake = intake;
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initPos = shooter.getEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.pusherPower(1);
    intake.spinWheel(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.pusherPower(0);
    intake.spinWheel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((shooter.getEncoder()+initPos) >= (42*5)){
      return true;
    }
    else{
    return false;
    }
  }
}
