// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.possibleDriveState;

public class AvancerXmCommand extends Command {
  /** Creates a new AvencerXmCommand. */
  private Drivetrain drivetrain;

  private PIDController pid1 = new PIDController(0.01, 0, 0);
  private PIDController pid2 = new PIDController(0.01, 0, 0);

  private double metre;
  private double target = 0.0;

  public AvancerXmCommand(Drivetrain drivetrain, double metre) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.metre = metre;
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetEncoder();
    drivetrain.switchMode(possibleDriveState.DRIVETANK);

    target = (metre * 17.25); // drivetrain.getEncoder()[1]+
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.driveTank(
        pid1.calculate(drivetrain.getAverageEncoder()[0], target),
        pid2.calculate(drivetrain.getAverageEncoder()[1], target));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.driveAllMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (3.0 > Math.abs(pid1.getPositionError()) && 3.0 > Math.abs(pid2.getPositionError())) {
      return true;
    } else {
      return false;
    }
  }
}
