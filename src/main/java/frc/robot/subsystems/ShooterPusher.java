// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class ShooterPusher extends SubsystemBase {
    private CANSparkMax pusherGauche;
  private CANSparkMax pusherDroit;
  private RelativeEncoder pusherEncoder;
  /** Creates a new ShooterPusher. */
  public ShooterPusher() {
    OperatorConstants deviceNumber = new OperatorConstants();
     pusherGauche = new CANSparkMax(deviceNumber.DeviceNumberPusherGauche, MotorType.kBrushless);
    pusherDroit = new CANSparkMax(deviceNumber.DeviceNumberPusherDroit, MotorType.kBrushless);
    pusherGauche.setInverted(false);
    pusherDroit.setInverted(false);
  }
  public double getEncoder(){
    return pusherEncoder.getPosition();
  }
  public void pusherPower(double power) {
    pusherDroit.set(power);
    pusherGauche.set(power);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
