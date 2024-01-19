// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class AvancerXmCommand extends PIDCommand {
  /** Creates a new Avancer1mCommand. */
  public AvancerXmCommand(Drivetrain drivetrain, double metre, int id) {

    super(
        // The controller that the command will use
        new PIDController(1, 0, 0),
        // This should return the measurement
        () -> drivetrain.getEncoder()[id],
        // This should return the setpoint (can also be a constant)
        () -> metre / 0.058,
        // This uses the output
        output -> {
          drivetrain.driveOneMotor(id, output);;
        });
    drivetrain.ActivateDrivetank();
    drivetrain.setDriveMode(true);

    addRequirements(drivetrain);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
