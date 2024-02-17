// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.util.Range;

public class SetRobotAngleCommand extends Command {
  private Drivetrain drivetrain;
  private Limelight limelight;
  private PIDController pid = new PIDController(0.035, 0.001, 0.001);

  private double angle;

  /** Creates a new SetRobotAngleCommand. */
  public SetRobotAngleCommand(Drivetrain drivetrain, Limelight limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    SmartDashboard.putBoolean("offset ready", false);
    addRequirements(drivetrain, limelight);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetGyroAngle();
    angle = -limelight.calculateShooterOffset();
    SmartDashboard.putNumber("target", angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Range.inRange(
        Math.abs(drivetrain.getGyroAngle()) - 1,
        Math.abs(drivetrain.getGyroAngle()) + 1,
        Math.abs(angle))) {
      SmartDashboard.putBoolean("offset ready", true);
    } else {
      SmartDashboard.putBoolean("offset ready", false);
      if (angle < 0) {
        drivetrain.driveRotation(
            -pid.calculate(Math.abs(drivetrain.getGyroAngle()), Math.abs(angle)));
      } else {
        drivetrain.driveRotation(pid.calculate(drivetrain.getGyroAngle(), angle));
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.driveRotation(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (angle - 5 < drivetrain.getGyroAngle() && angle + 5 > drivetrain.getGyroAngle()) {
      return true;
    } else {
      return false;
    }
  }
}
