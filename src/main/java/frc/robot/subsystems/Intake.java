// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class Intake extends SubsystemBase {
  /** Creates a new intake. */
  private CANSparkMax intakeDroit;
  private CANSparkMax intakeGauche;
  private CANSparkMax intakePivot;

  DigitalInput outlimitSwitch = new DigitalInput(0);
  DigitalInput inlimitSwitch = new DigitalInput(1);
  public Intake() {
    OperatorConstants deviceNumber = new OperatorConstants();
    intakeDroit = new CANSparkMax(deviceNumber.DeviceNumberIntakeDroit, MotorType.kBrushless);
    intakeGauche = new CANSparkMax(deviceNumber.DeviceNumberIntakeGauche, MotorType.kBrushless);
    intakePivot = new CANSparkMax(deviceNumber.DeviceNumberIntakePivot, MotorType.kBrushless);

    intakeDroit.setInverted(false);
    intakeGauche.setInverted(false);
    intakePivot.setInverted(false);
  }
  public void spinWheel(double power){
    intakeDroit.set(power);
    intakeGauche.set(power);
  }
  public void moveIntake(double power){
    if(inlimitSwitch.get() && power < 0){
      intakePivot.set(0);
    }
    else if(outlimitSwitch.get() && power > 0){
      intakePivot.set(0);
    }
    else{
      intakePivot.set(power);
    }
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
