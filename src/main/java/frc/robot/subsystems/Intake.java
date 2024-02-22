// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DIGITAL;
import frc.util.MathHelper;
import frc.util.Range;
public class Intake extends SubsystemBase {
  /** Creates a new intake. */
  private CANSparkMax intakeDroit;

  private CANSparkMax intakeGauche;

  private CANSparkMax intakePivot;

  DigitalInput intakeLimitIn = new DigitalInput(DIGITAL.INTAKE_LIMIT_SWITCH_IN);
  DigitalInput intakeLimitOut = new DigitalInput(DIGITAL.INTAKE_LIMIT_SWITCH_OUT);

  private boolean wantedInside = false;
  private boolean asBeenPressed = false;

  public Intake() {
    intakeDroit = new CANSparkMax(CAN.DeviceNumberIntakeDroit, MotorType.kBrushless);
    intakeGauche = new CANSparkMax(CAN.DeviceNumberIntakeGauche, MotorType.kBrushless);
    intakePivot = new CANSparkMax(CAN.DeviceNumberIntakePivot, MotorType.kBrushless);

    intakeDroit.setInverted(false);
    intakeGauche.setInverted(false);
    intakePivot.setInverted(false);
    intakePivot.setSmartCurrentLimit(30, 20);
    intakeDroit.setSmartCurrentLimit(20);
    intakeGauche.setSmartCurrentLimit(20);
    intakePivot.setIdleMode(IdleMode.kBrake);

    intakeDroit.burnFlash();
    intakeGauche.burnFlash();
    intakePivot.burnFlash();
  }

  public void spinWheel(double power) {
    intakeDroit.set(power / 2);
    intakeGauche.set(power / 2);
  }
  public boolean getAsBeenPressed(){
    return asBeenPressed;
  }
  public void asBeenPressed(){
    asBeenPressed = true;
  }

  public double getVelocity(){
    return intakePivot.getEncoder().getVelocity();
  }

  public void moveIntake(double power) {
   /*if(getIntakeLimitIn() || getIntakeLimitOut()){
      intakePivot.set(0);
    }
    else if(getIntakeLimitIn() && getIntakeLimitOut()){
      intakePivot.set(0);
    }
   else if(getIntakeLimitIn() && power > 0){
      intakePivot.set(power);
    }else if(getIntakeLimitOut() && power < 0){
      intakePivot.set(power);
    }
    
    else{
      intakePivot.set(power);
    }
  }*/
  if(getIntakeLimitIn()){
    power = Range.coerce(0, 1, power);

    intakePivot.set(power);
  }

  else if(getIntakeLimitOut()){
    power = Range.coerce(-1, 0, power);
 
    intakePivot.set(power);
  }

  }
  public boolean wantedInside(){
    return wantedInside;
  }

  public void setState(boolean isIn){
    this.wantedInside = isIn;
  }

  public boolean getIntakeLimitIn() {
    return intakeLimitIn.get();
  }

  public boolean getIntakeLimitOut(){
    return intakeLimitOut.get();
  }

  public void resetEncoder() {
    intakePivot.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("limit in", getIntakeLimitIn());
    SmartDashboard.putBoolean("limit out", getIntakeLimitOut());
    SmartDashboard.putNumber("IntakeEncoder", intakePivot.getEncoder().getPosition());

   
    // This method will be called once per scheduler run
  }
}
