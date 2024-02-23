// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.AngleShooter;
import frc.robot.subsystems.Limelight;
import frc.util.MathHelper;
import frc.util.Range;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetShooterAngleCommand extends PIDCommand {
  /** Creates a new SetShooterAngleCommand. */
  public SetShooterAngleCommand(AngleShooter angleShooter, Limelight limelight) {
    super(
        // The controller that the command will use
        new PIDController(1, 0.5, 0.5), // TODO change value
        // This should return the measurement
        () -> angleShooter.getEncoderValue(),
        // This should return the setpoint (can also be a constant)

        () -> MathHelper.map(limelight.calculateShooterAngle(), 60, 78.85, angleShooter.getEncoderMax(), angleShooter.getEncoderMin()),
        // sur l'angle calculer

        // This uses the output
        output -> {
          System.out.println(-output);
          angleShooter.setPower(-output);
         SmartDashboard.putNumber("target encoder", MathHelper.map(limelight.calculateShooterAngle(), 52.78, 78.85, angleShooter.getEncoderMax(), angleShooter.getEncoderMin()));
          if (output > -0.000001 && output < 0.000001) {
            SmartDashboard.putBoolean("Angle Ready", true);
          } else {
            SmartDashboard.putBoolean("Angle Ready", false);
          }
        });
    SmartDashboard.putBoolean("Angle Ready", false);

    addRequirements(angleShooter, limelight);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
