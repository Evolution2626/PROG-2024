// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.shooterPossibleState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetAmpStateCommand extends InstantCommand {
  private Shooter shooter;

  public SetAmpStateCommand(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(shooter.getShooterState() == shooterPossibleState.AMP) {
      shooter.setShooterState(shooterPossibleState.OFF);
    }else{
      shooter.setShooterState(shooterPossibleState.AMP);
    }
  }
}
