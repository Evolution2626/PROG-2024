// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetShooterSpeedCommand extends Command {
  private Shooter shooter;
  private double speed = 5000;
  private double kVBas;
  private double kVHaut;
  private PIDController pidControllerBasRPM = new PIDController(0, 0, 0);
  private PIDController pidControllerHautRPM = new PIDController(0, 0, 0);

  /** Creates a new SetShooterSpeedCommand. */
  public SetShooterSpeedCommand(Shooter shooter) {
    this.shooter = shooter;
    kVBas = 0.00018;
    kVHaut = 0.00018;
    SmartDashboard.putBoolean("Speed Ready", false);
    addRequirements(shooter);
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
    shooter.pusherPower(1);
    if(shooter.wantedShooting()){
      shooter.pusherPower(1);
      shooter.shooterPower(
        (pidControllerBasRPM.calculate(shooter.getVelocityBas(), speed) + (kVBas * speed)),
        (pidControllerHautRPM.calculate(shooter.getVelocityHaut(), speed) + (kVHaut * speed)));
    }
    if(shooter.wantedAmp() && !shooter.wantedShooting()){
      shooter.pusherPower(1);
      shooter.shooterPower(0.2, 0.2);
    }
    else{
      shooter.pusherPower(0);
      shooter.shooterPower(0.05, 0.05);

      

    }

    if (shooter.getVelocityHaut() > speed - 100 && shooter.getVelocityBas() > speed - 100) {
      SmartDashboard.putBoolean("Speed Ready", true);

    } else {
      SmartDashboard.putBoolean("Speed Ready", false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //shooter.pusherPower(0);
    //shooter.shooterPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(shooter.getVelocityBas() > speed-100 && shooter.getVelocityHaut() > speed-100){
      return true;
    }
    return false;
  }
}
