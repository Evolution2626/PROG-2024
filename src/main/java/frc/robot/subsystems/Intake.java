// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DIGITAL;
import frc.util.Range;

public class Intake extends SubsystemBase {
  /** Creates a new intake. */
  private CANSparkMax intakeDroit;

  private CANSparkMax intakeGauche;

  private CANSparkMax intakePivot;

  DigitalInput intakeLimitIn = new DigitalInput(DIGITAL.INTAKE_LIMIT_SWITCH_IN);
  DigitalInput intakeLimitOut = new DigitalInput(DIGITAL.INTAKE_LIMIT_SWITCH_OUT);

  private boolean wantedInside = false;

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
    intakeDroit.setIdleMode(IdleMode.kBrake);
    intakeGauche.setIdleMode(IdleMode.kBrake);

    intakeDroit.burnFlash();
    intakeGauche.burnFlash();
    intakePivot.burnFlash();
  }

  public void spinWheel(double power) {
    intakeDroit.set(power*0.75 );
    intakeGauche.set(power*0.75 );
  }

  
  public void resetWheelEncoder(){
    intakeDroit.getEncoder().setPosition(0);
  }


  public double getVelocity() {
    return intakePivot.getEncoder().getVelocity();
  }
  public double getWheelEncoder(){
    return intakeDroit.getEncoder().getPosition();
  }

  public double getPosition() {
    return intakePivot.getEncoder().getPosition();
  }

  public void moveIntake(double power) {
    if (getIntakeLimitIn()) {
      power = Range.coerce(0, 1, power);

      SmartDashboard.putNumber("Power In", power);

      intakePivot.set(power);
      intakePivot.getEncoder().setPosition(0);
    } else if (getIntakeLimitOut()) {
      power = Range.coerce(-1, 0, power);

      SmartDashboard.putNumber("Power Out", power);

      intakePivot.set(power);
    }
  }

  public boolean wantedInside() {
    return wantedInside;
  }

  public double intakeCurveFunction(double c, double b, double a){
    SmartDashboard.putNumber("Curve Function", (b / Math.abs(b)) * (Math.pow(c / (getPosition() - a), 2)) - b);
    return (b / Math.abs(b)) * (Math.pow(c / (getPosition() - a), 2)) - b;

  }
  public void setState(boolean isIn) {
    this.wantedInside = isIn;
  }

  public boolean getIntakeLimitIn() {
    return intakeLimitIn.get();
  }

  public boolean getIntakeLimitOut() {
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
