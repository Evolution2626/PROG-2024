// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class AvancerXmCommand extends Command {
  /** Creates a new AvencerXmCommand. */
  Drivetrain drivetrain;
  private PIDController pid = new PIDController(0.0025,0,0); 
  private double metre;
  private double target;
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
    drivetrain.ActivateDrivetank();
    drivetrain.setDriveMode(true);
    target = (drivetrain.getEncoder()[1]+((metre / (( 0.1016 * Math.PI))*(12.0 / 66.0)*42.0)));
    SmartDashboard.putNumber("target", target);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("encoder", drivetrain.getEncoder()[0]);
    drivetrain.driveAllMotor(pid.calculate(drivetrain.getEncoder()[0], target));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.driveAllMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(target - 10 < drivetrain.getEncoder()[0] && target + 10 > drivetrain.getEncoder()[0]){
      System.out.println("dsljkfhjkasdfhsdalfhsd,fjdsahfsdahflsdkfhsdklfhdsfklsdhflksdfhsdklcsdklcnsdklcvsdvlkdsnh");
      return true;
    }
    else{
    return false;
    }
  }
}
