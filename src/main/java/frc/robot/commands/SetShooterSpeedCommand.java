// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetShooterSpeedCommand extends Command {
  private Shooter shooter;
  private double speed;
  private double kV;
  private PIDController pidControllerDroitRPM = new PIDController(0.1, 0.1, 0);
  private PIDController pidControllerGaucheRPM = new PIDController(0.1, 0.1, 0);
  
  /** Creates a new SetShooterSpeedCommand. */
  public SetShooterSpeedCommand(Shooter shooter, double speed) {
    this.shooter = shooter;
    this.speed = speed;
    kV = 0.1;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.shooterPower((pidControllerDroitRPM.calculate(shooter.getVelocityDroit(), speed) + (kV*speed)), (pidControllerGaucheRPM.calculate(shooter.getVelocityGauche(), speed)+ (kV*speed)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.shooterPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
