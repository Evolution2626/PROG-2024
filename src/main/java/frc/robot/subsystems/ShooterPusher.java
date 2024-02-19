// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;

public class ShooterPusher extends SubsystemBase {
  private CANSparkMax pusher;
  private RelativeEncoder pusherEncoder;

  /** Creates a new ShooterPusher. */
  public ShooterPusher() {
    pusher = new CANSparkMax(CAN.DeviceNumberPusher, MotorType.kBrushless);
    pusher.setInverted(false);
    pusherEncoder = pusher.getEncoder();
    pusher.setSmartCurrentLimit(30);
  }

  public double getEncoder() {
    return pusherEncoder.getPosition();
  }

  public void pusherPower(double power) {
    pusher.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
