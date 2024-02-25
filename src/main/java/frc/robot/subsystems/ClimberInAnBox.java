// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.PCM;

public class ClimberInAnBox extends SubsystemBase {
  private TalonSRX climberDroit;
  private TalonSRX climberGauche;
  private DoubleSolenoid piston1;
  private boolean ratchetActivated = false;
  private boolean climberOut = false;

  /** Creates a new ClimberInAnBox. */
  public ClimberInAnBox() {
    climberDroit = new TalonSRX(CAN.DeviceNumberClimberDroit);
    climberGauche = new TalonSRX(CAN.DeviceNumberClimberGauche);

    climberDroit.setInverted(false);
    climberGauche.setInverted(false);

    climberDroit.configPeakCurrentLimit(40, 5);
    climberDroit.configPeakCurrentDuration(200, 5);
    climberDroit.configContinuousCurrentLimit(30, 5);
    climberDroit.enableCurrentLimit(true);

    climberGauche.configPeakCurrentLimit(40, 5);
    climberGauche.configPeakCurrentDuration(200, 5);
    climberGauche.configContinuousCurrentLimit(30, 5);
    climberGauche.enableCurrentLimit(true);

    piston1 =
        new DoubleSolenoid(
            1,
            PneumaticsModuleType.REVPH,
            PCM.PISTON_CLIMBER_FORWARD_1,
            PCM.PISTON_CLIMBER_REVERSE_1);

    piston1.set(DoubleSolenoid.Value.kReverse);
  }

  public void climb(double droit, double gauche) {
    climberDroit.set(ControlMode.PercentOutput, droit);
    climberGauche.set(ControlMode.PercentOutput, gauche);
  }
  public void setClimberOut(boolean isOut){
    climberOut = isOut;
  }
  public boolean getclimberOut(){
    return climberOut;
  }

  public void activateRatchet() {
    ratchetActivated = true;
    piston1.set(DoubleSolenoid.Value.kForward);
  }
  public boolean isRatchetActivated(){
    return ratchetActivated;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
