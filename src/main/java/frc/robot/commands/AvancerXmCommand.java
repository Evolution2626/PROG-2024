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
  private PIDController pid = new PIDController(0.0075,0,0); 
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
    drivetrain.ActivateDrivetank();
    drivetrain.setDriveMode(true);
    target = (((metre / ( 0.1016 * Math.PI))*(12.0 / 66.0))*42);//drivetrain.getEncoder()[1]+
    SmartDashboard.putNumber("target", target);
   
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.driveAllMotor(pid.calculate(drivetrain.getEncoder()[1], target));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.driveAllMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(3.0 > Math.abs(pid.getPositionError())){
      System.out.println("dsljkfhjkasdfhsdalfhsd,fjdsahfsdahflsdkfhsdklfhdsfklsdhflksdfhsdklcsdklcnsdklcvsdvlkdsnh");
      
      return true;
    }
    else{
    return false;
    }
  }
}
