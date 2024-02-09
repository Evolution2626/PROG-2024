// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.AngleShooter;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetShooterAngleCommand extends PIDCommand {
  /** Creates a new SetShooterAngleCommand. */
  private AngleShooter angleShooter;

  Limelight limelight;

  public SetShooterAngleCommand(AngleShooter angleShooter, Limelight limelight) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0), // TODO change value
        // This should return the measurement
        () -> angleShooter.getEncoderValue(),
        // This should return the setpoint (can also be a constant)
        () ->
            ((limelight.calculateShooterAngle()
                        * (angleShooter.getEncoderMax() - angleShooter.getEncoderMin()))
                    / 30)
                + angleShooter
                    .getEncoderMin() // un produit  croiser pour calculer les valeur d'encoder basee
        // sur l'angle calculer
        ,
        // This uses the output
        output -> {
          angleShooter.setPower(output);
          if (output > -0.1 && output < 0.1) {
            SmartDashboard.putBoolean("Angle Ready", true);
          } else {
            SmartDashboard.putBoolean("Angle Ready", false);
          }
        });
    this.angleShooter = angleShooter;
    this.limelight = limelight;
    addRequirements(angleShooter, limelight);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
