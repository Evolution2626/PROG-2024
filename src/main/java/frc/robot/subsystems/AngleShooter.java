// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class AngleShooter extends SubsystemBase {
  private CANSparkMax shooterAngle;
  private DutyCycleEncoder shooterAngleEncoder = new DutyCycleEncoder(0);

  private double encoderOffset = 0.0;

  /** Creates a new AngleShooter. */
  public AngleShooter() {
    OperatorConstants deviceNumber = new OperatorConstants();

    shooterAngle = new CANSparkMax(deviceNumber.DeviceNumberShooterAngle, MotorType.kBrushless);

    shooterAngle.setInverted(false);
  }

  public double getEncoderValue() {

    return shooterAngleEncoder.getAbsolutePosition() - encoderOffset;
  }

  public void setPower(double power) {
    shooterAngle.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter angle", shooterAngleEncoder.getAbsolutePosition());
  }
}
