// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AngleShooter;
import frc.robot.subsystems.Limelight;
import frc.util.MathHelper;

public class SetShooterAngleCommand extends Command {
  private Limelight limelight;
  private AngleShooter angleShooter;

  private PIDController anglePID = new PIDController(1, 0.5, 0.5);

  private double output;

  /** Creates a new SetShooterAngleCommand. */
  public SetShooterAngleCommand(Limelight limelight, AngleShooter angleShooter) {
    this.limelight = limelight;
    this.angleShooter = angleShooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, angleShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelight.getIsTargetFound()) {
      output =
          anglePID.calculate(
              angleShooter.getEncoderValue(),
              MathHelper.map(
                  limelight.calculateShooterAngle(),
                  60,
                  76.85,
                  angleShooter.getEncoderMax(),
                  angleShooter.getEncoderMin()));
    } else {
      output = anglePID.calculate(angleShooter.getEncoderValue(), angleShooter.getEncoderMin());
    }

    angleShooter.setPower(-output);
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
