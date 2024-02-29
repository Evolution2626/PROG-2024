// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.CAN;
import frc.robot.Constants.PCM;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Amp extends SubsystemBase {
  private DoubleSolenoid piston;
  private boolean position;
  /** Creates a new Amp. */
  public Amp() {
    piston = new DoubleSolenoid(49, PneumaticsModuleType.REVPH, PCM.PISTON_AMP_SHOOTER_FORWARD,PCM.PISTON_AMP_SHOOTER_REVERSE);

      piston.set(DoubleSolenoid.Value.kReverse);

  }
  public boolean getPosition() {
    return position;
  }
  public void setPosition(boolean out) {
    if (out) {
      position = true;
      piston.set(DoubleSolenoid.Value.kForward);
    } else {
      piston.set(DoubleSolenoid.Value.kReverse);
      position = false;
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
