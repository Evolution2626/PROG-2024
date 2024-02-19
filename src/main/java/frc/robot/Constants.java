// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class CAN {
    public static final int DeviceNumberAvantDroit = 3;
    public static final int DeviceNumberAvantGauche = 8;
    public static final int DeviceNumberArriereDroit = 7;
    public static final int DeviceNumberArriereGauche = 6;

    public static final int DeviceNumberClimberDroit = 99;
    public static final int DeviceNumberClimberGauche = 98;

    public static final int DeviceNumberShooterHaut = 42;
    public static final int DeviceNumberShooterBas = 41;

    public static final int DeviceNumberPusher = 43;

    public static final int DeviceNumberShooterAngle = 40;

    public static final int DeviceNumberIntakeDroit = 44;
    public static final int DeviceNumberIntakeGauche = 12;
    public static final int DeviceNumberIntakePivot = 5;

    public static final int DeviceNumberAmpShooter = 13;
  }

  public static class PCM {
    public static final int PISTON_FORWARD = 0;
    public static final int PISTON_REVERSE = 1;

    public static final int PISTON_CLIMBER_FORWARD_1 = 2;
    public static final int PISTON_CLIMBER_REVERSE_1 = 3;

    public static final int PISTON_CLIMBER_FORWARD_2 = 4;
    public static final int PISTON_CLIMBER_REVERSE_2 = 5;

    public static final int PISTON_AMP_SHOOTER_FORWARD = 6;
    public static final int PISTON_AMP_SHOOTER_REVERSE = 7;
  }

  public static class DIGITAL {
    public static final int INTAKE_LIMIT_SWITCH_OUT = 6;
    public static final int INTAKE_LIMIT_SWITCH_IN = 9;
  }
}
