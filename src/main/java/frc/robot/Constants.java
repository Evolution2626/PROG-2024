// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public final int DeviceNumberAvantDroit = 3;
    public final int DeviceNumberAvantGauche = 33;
    public final int DeviceNumberArriereDroit = 14;
    public final int DeviceNumberArriereGauche = 7;

    public final int DeviceNumberClimberDroit = 17;
    public final int DeviceNumberClimberGauche = 15;

    public final int DeviceNumberShooterGauche = 102;
    public final int DeviceNumberShooterDroit = 103;

    public final int DeviceNumberPusherGauche = 104;
    public final int DeviceNumberPusherDroit = 105;

    public final int DeviceNumberShooterAngle = 106;

    public final int DeviceNumberIntakeDroit = 70;
    public final int DeviceNumberIntakeGauche = 90;
    public final int DeviceNumberIntakePivot = 109;

    public final int DeviceNumberAmpShooter = 110;
  }

  public static class PCM {
    public final int PISTON_FORWARD = 0;
    public final int PISTON_REVERSE = 1;

    public final int PISTON_CLIMBER_FORWARD_1 = 2;
    public final int PISTON_CLIMBER_REVERSE_1 = 3;

    public final int PISTON_CLIMBER_FORWARD_2 = 4;
    public final int PISTON_CLIMBER_REVERSE_2 = 5;

    public final int PISTON_AMP_SHOOTER_FORWARD = 6;
    public final int PISTON_AMP_SHOOTER_REVERSE = 7;
  }

  public static class DIGITAL {
    public final int INTAKE_LIMIT_SWITCH_OUT = 6;
    public final int INTAKE_LIMIT_SWITCH_IN = 7;
  }

  public static class DriveConstants {
    public static final double ksVolts = 0.26571;
    public static final double kvVoltSecondsPerMeter = 1.4492;
    public static final double kaVoltSecondsSquaredPerMeter = 2.3363;

    public static final double velocityConversionFactor =
        (12.0 / 66.0) * 4.0 * Math.PI * 2.54 / 6000.0;
    public static final double positionConversionFactor =
        (12.0 / 66.0) * 4.0 * Math.PI * 2.45 / 100.0;

    public static final double kPDriveVel = 0.0020707;

    public static final double kTrackwidthMeters = 0.58;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
