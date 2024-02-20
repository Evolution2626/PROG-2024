// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DIGITAL;

public class Intake extends SubsystemBase {
  /** Creates a new intake. */
  private CANSparkMax intakeDroit;

  private CANSparkMax intakeGauche;

  private CANSparkMax intakePivot;

  DigitalInput intakeLimitIn = new DigitalInput(DIGITAL.INTAKE_LIMIT_SWITCH_IN);

  public Intake() {
    intakeDroit = new CANSparkMax(CAN.DeviceNumberIntakeDroit, MotorType.kBrushless);
    intakeGauche = new CANSparkMax(CAN.DeviceNumberIntakeGauche, MotorType.kBrushless);
    intakePivot = new CANSparkMax(CAN.DeviceNumberIntakePivot, MotorType.kBrushless);

    intakeDroit.setInverted(false);
    intakeGauche.setInverted(false);
    intakePivot.setInverted(false);
    intakePivot.setSmartCurrentLimit(10);
    intakeDroit.setSmartCurrentLimit(30);
    intakeGauche.setSmartCurrentLimit(30);

    intakeDroit.burnFlash();
    intakeGauche.burnFlash();
    intakePivot.burnFlash();
  }

  public void spinWheel(double power) {

    intakeDroit.set(power / 2);
    intakeGauche.set(power / 2);
  }

  public void moveIntake(double power) {

    intakePivot.set(power);
  }

  public boolean getIntakeLimitIn() {
    return intakeLimitIn.get();
  }

  public RelativeEncoder getEncoder() {
    return intakePivot.getEncoder();
  }

  public void resetEncoder() {
    intakePivot.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("IntakeEncoder", intakePivot.getEncoder().getPosition());

    if (intakeLimitIn.get()) {
      intakePivot.getEncoder().setPosition(0);
    }
    // This method will be called once per scheduler run
  }
}
