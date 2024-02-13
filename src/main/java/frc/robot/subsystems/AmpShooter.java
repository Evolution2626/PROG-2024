// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpShooter extends SubsystemBase {
  private CANSparkMax ampShooterMotor;
  private DoubleSolenoid piston;

  /** Creates a new AmpShooter. */
  public AmpShooter() {
    piston =
        new DoubleSolenoid(
            1,
            PneumaticsModuleType.CTREPCM,
            Constants.PCM.PISTON_AMP_SHOOTER_FORWARD,
            Constants.PCM.PISTON_AMP_SHOOTER_REVERSE);

    ampShooterMotor = new CANSparkMax(Constants.OperatorConstants.DeviceNumberAmpShooter, MotorType.kBrushless);
    ampShooterMotor.setInverted(false);
  }

  public void setPosition(boolean out) {
    if (out) {
      piston.set(DoubleSolenoid.Value.kForward);
    } else {
      piston.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void setWheelPower(double power) {
    ampShooterMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
