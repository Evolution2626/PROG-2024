// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;



public class TournerXdegCommand extends Command {
  PIDController pid = new PIDController(0.035, 0.001, 0.001); // TODO put value
  Drivetrain drivetrain;
  double degree;
  /** Creates a new Tourner. */
  public TournerXdegCommand(Drivetrain drivetrain, double degree) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.degree = degree;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetGyroAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (degree < 0) {
      drivetrain.driveRotation(
          -pid.calculate(Math.abs(drivetrain.getGyroAngle()), Math.abs(degree)));
    } else {
      drivetrain.driveRotation(pid.calculate(drivetrain.getGyroAngle(), degree));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(degree - 5 < drivetrain.getGyroAngle() && degree + 5 > drivetrain.getGyroAngle()){
      return true;
    }
    else{
    return false;
    }
  }
}
