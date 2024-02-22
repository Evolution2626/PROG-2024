// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;

public class AngleShooter extends SubsystemBase {
  private CANSparkMax shooterAngle;
  private DutyCycleEncoder shooterAngleEncoder = new DutyCycleEncoder(0);

  private double encoderMax = 0.39;
  private double encoderMin = 0.239;

  /** Creates a new AngleShooter. */
  public AngleShooter() {
    shooterAngle = new CANSparkMax(CAN.DeviceNumberShooterAngle, MotorType.kBrushless);

    shooterAngle.setInverted(false);
    shooterAngle.setSmartCurrentLimit(10);
    shooterAngle.burnFlash();
  }

  public double getEncoderValue() {

    return shooterAngleEncoder.getAbsolutePosition();
  }

  public double getEncoderMax() {
    return encoderMax;
  }

  public double getEncoderMin() {
    return encoderMin;
  }

  public void setPower(double power) {
    shooterAngle.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter angle current encoder", shooterAngleEncoder.getAbsolutePosition());
    // for debug
  }
}
