// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

public class AvencerXmCommand extends Command {
  /** Creates a new AvencerXmCommand. */
  Drivetrain drivetrain;
  private PIDController pid = new PIDController(1,0,0); 
  private double metre;
  private double target;
  public AvencerXmCommand(Drivetrain drivetrain, double metre) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.metre = metre;
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("asfnsdljfbsdfk");
    drivetrain.ActivateDrivetank();
    drivetrain.setDriveMode(true);
    target = (drivetrain.getEncoder()[0]+(metre * ((12.0 / 66.0) * 0.1016 * Math.PI * 2.45)));
    System.out.println("bobobobobobobobo");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    SmartDashboard.putNumber("ëncoder0", 0);
    drivetrain.driveAllMotor(pid.calculate(drivetrain.getEncoder()[0], target));
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
