// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.shooterPossibleState;

public class SetShooterSpeedCommand extends Command {
  private Shooter shooter;
  private Amp amp;
  private double speed = 5600;
  private double kVBas;
  private double kVHaut;
  private PIDController pidControllerBasRPMSpeaker = new PIDController(0, 0, 0);
  private PIDController pidControllerHautRPMSpeaker = new PIDController(0, 0, 0);

  /** Creates a new SetShooterSpeedCommand. */
  public SetShooterSpeedCommand(Shooter shooter, Amp amp) {
    this.amp = amp;
    this.shooter = shooter;
    kVBas = 0.000161;
    kVHaut = 0.000161;
    addRequirements(shooter, amp);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // shooter.shooterPower(1, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // shooter.pusherPower(1);
    if (shooter.getShooterState() == shooterPossibleState.SPEAKER) {
      amp.setPosition(true);
      shooter.pusherPower(1);
      /*shooter.shooterPower(
          (pidControllerBasRPMSpeaker.calculate(shooter.getVelocityBas(), speed) + (kVBas * speed)),
          (pidControllerHautRPMSpeaker.calculate(shooter.getVelocityHaut(), speed)
              + (kVHaut * speed)));*/
              shooter.shooterPower(0.9, 0.9);
    } else if (shooter.getShooterState() == shooterPossibleState.AMP) {
      amp.setPosition(true);
      shooter.pusherPower(1);
      shooter.shooterPower(0.2, 0.2);
    } else if (shooter.getShooterState() == shooterPossibleState.OFF) {
      amp.setPosition(false);
      shooter.pusherPower(0);
      shooter.shooterPower(0.05, 0.05);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // shooter.pusherPower(0);
    // shooter.shooterPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
