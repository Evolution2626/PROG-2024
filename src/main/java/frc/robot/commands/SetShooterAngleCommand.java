// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
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
        new PIDController(0, 0, 0),
        // This should return the measurement
        () -> angleShooter.getEncoder(),
        // This should return the setpoint (can also be a constant)
        () ->
            (angleShooter.encoderZero()
                + ((limelight.calculateShooterAngle()
                    * 8.57))), // TODO adjust 8.57 based on gearbox and encoder
        // This uses the output
        output -> {
          angleShooter.setPower(output);
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
