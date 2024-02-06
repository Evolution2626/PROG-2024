// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIGITAL;
import frc.robot.Constants.OperatorConstants;

public class Intake extends SubsystemBase {
  /** Creates a new intake. */
  DIGITAL degitalInput = new DIGITAL();

  OperatorConstants deviceNumber = new OperatorConstants();

  private CANSparkMax intakeDroit;

  private CANSparkMax intakeGauche;
  private CANSparkMax intakePivot;

  DigitalInput intakeLimitOut = new DigitalInput(degitalInput.INTAKE_LIMIT_SWITCH_OUT);
  DigitalInput intakeLimitIn = new DigitalInput(degitalInput.INTAKE_LIMIT_SWITCH_IN);

  public Intake() {
    intakeDroit = new CANSparkMax(deviceNumber.DeviceNumberIntakeDroit, MotorType.kBrushless);
    intakeGauche = new CANSparkMax(deviceNumber.DeviceNumberIntakeGauche, MotorType.kBrushless);
    intakePivot = new CANSparkMax(deviceNumber.DeviceNumberIntakePivot, MotorType.kBrushless);

    intakeDroit.setInverted(false);
    intakeGauche.setInverted(false);
    intakePivot.setInverted(false);
  }

  public void spinWheel(double power) {
    intakeDroit.set(power);
    intakeGauche.set(power);
  }

  public void moveIntake(double power) {
    if (intakeLimitIn.get() && power < 0) {
      intakePivot.set(0);
    } else if (intakeLimitOut.get() && power > 0) {
      intakePivot.set(0);
    } else {
      intakePivot.set(power);
    }
  }

  public boolean getIntakeLimitOut() {
    return intakeLimitOut.get();
  }

  public boolean getIntakeLimitIn() {
    return intakeLimitIn.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
