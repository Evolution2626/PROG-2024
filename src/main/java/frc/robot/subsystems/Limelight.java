package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.ControlMode.*;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private NetworkTable networkTable;

  private String tableName;

  private Boolean isConnected;

  private double _hearBeatPeriod = 0.1;

  private double[] lastPosition;

  public Translation2d robotPos;
  public Translation2d setpoint;

  class PeriodicRunnable implements java.lang.Runnable {
    public void run() {
      resetPilelineLatency();
      try {
        Thread.sleep(50);
      } catch (InterruptedException e) {

        e.printStackTrace();
      }
      if (getPipelineLatency() == 0.0) {
        isConnected = false;
      } else {
        isConnected = true;
      }
    }
  }

  Notifier _hearBeat = new Notifier(new PeriodicRunnable());

  /** Using the Default Lime Light NT table */
  public Limelight() {
    tableName = "limelight";
    networkTable = NetworkTableInstance.getDefault().getTable(tableName);
    _hearBeat.startPeriodic(_hearBeatPeriod);
    setLEDMode(0);
  }

  /** If you changed the name of your Lime Light tell Me the New Name */
  public Limelight(String tableName) {
    this.tableName = tableName;
    networkTable = NetworkTableInstance.getDefault().getTable(tableName);
    _hearBeat.startPeriodic(_hearBeatPeriod);
  }

  /** Send an instance of the NetworkTabe */
  public Limelight(NetworkTable table) {
    networkTable = table;
    _hearBeat.startPeriodic(_hearBeatPeriod);
  }

  public double getPipelineLatency() {
    NetworkTableEntry tl = networkTable.getEntry("tl");
    double l = tl.getDouble(0.0);
    return l;
  }

  private void resetPilelineLatency() {
    networkTable.getEntry("tl").setValue(0.0);
  }

  public boolean isConnected() {
    return isConnected; // test
  }

  /**
   * tv Whether the limelight has any valid targets (0 or 1)
   *
   * @return
   */
  public boolean getIsTargetFound() {
    NetworkTableEntry tv = networkTable.getEntry("tv");
    double v = tv.getDouble(0);
    if (v == 0.0f) {
      return false;
    } else {
      return true;
    }
  }

  /**
   * tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
   *
   * @return
   */
  public double getdegRotationToTarget() {
    NetworkTableEntry tx = networkTable.getEntry("tx");
    double x = tx.getDouble(0.0);
    return x;
  }

  /**
   * ty Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
   *
   * @return
   */
  public double getdegVerticalToTarget() {
    NetworkTableEntry ty = networkTable.getEntry("ty");
    double y = ty.getDouble(0.0);
    return y;
  }

  /**
   * ta Target Area (0% of image to 100% of image)
   *
   * @return
   */
  public double getTargetArea() {
    NetworkTableEntry ta = networkTable.getEntry("ta");
    double a = ta.getDouble(0.0);
    return a;
  }

  /**
   * ts Skew or rotation (-90 degrees to 0 degrees)
   *
   * @return
   */
  public double getSkew_Rotation() {
    NetworkTableEntry ts = networkTable.getEntry("ts");
    double s = ts.getDouble(0.0);
    return s;
  }

  /**
   * botpose Get position of the robot on the field
   *
   * @return
   */
  public double[] getRobotPosition() {

    NetworkTableEntry botpose = networkTable.getEntry("botpose");
    double[] position = botpose.getDoubleArray(lastPosition);

    if (position != null && position.length == 6) {
      lastPosition = position;
    }
    return lastPosition;
  }

  /**
   * target distance from the robot in meters
   *
   * @return
   */
  /*
  public double getTargetDistance(double targetHeight){
    return (targetHeight - Constants.GAMES_CONSTANTS.CAMERA_HEIGHT) / Math.tan(Math.toRadians(getdegVerticalToTarget()));
  }*/

  // Setters

  /**
   * LedMode Sets limelight’s LED state
   *
   * <p>kon koff kblink
   *
   * @param i
   */
  public void setLEDMode(int i) {
    networkTable.getEntry("ledMode").setValue(i);
  }

  public double calculateShooterOffset() {
    double offset = getdegRotationToTarget();

    return offset;
  }

  public double calculateShooterAngle() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 35.0;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 8;

    // distance from the target to the floor
    double goalHeightInches = 55;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // calculate distance
    double distanceFromLimelightToGoalInches =
        (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    SmartDashboard.putNumber(
        "shooterAngle", Math.atan(78 / distanceFromLimelightToGoalInches) / Math.PI * 180.0 - 30);
    return Math.atan(78 / distanceFromLimelightToGoalInches) / Math.PI * 180.0 - 30;
  }

  /**
   * Returns current LED mode of the Lime Light
   *
   * @return LedMode
   */
  public LedMode getLEDMode() {
    NetworkTableEntry ledMode = networkTable.getEntry("ledMode");
    double led = ledMode.getDouble(0.0);
    LedMode mode = LedMode.getByValue(led);
    return mode;
  }

  /**
   * camMode Sets limelight’s operation mode
   *
   * <p>kvision kdriver (Increases exposure, disables vision processing)
   *
   * @param camMode
   */
  public void setCamMode(CamMode camMode) {
    networkTable.getEntry("camMode").setValue(camMode.getValue());
  }

  /**
   * Returns current Cam mode of the Lime Light
   *
   * @return CamMode
   */
  public CamMode getCamMode() {
    NetworkTableEntry camMode = networkTable.getEntry("camMode");
    double cam = camMode.getDouble(0.0);
    CamMode mode = CamMode.getByValue(cam);
    return mode;
  }

  /**
   * pipeline Sets limelight’s current pipeline
   *
   * <p>0 . 9 Select pipeline 0.9
   *
   * @param pipeline
   */
  public void setPipeline(Integer pipeline) {
    if (pipeline < 0) {
      pipeline = 0;
      throw new IllegalArgumentException("Pipeline can not be less than zero");
    } else if (pipeline > 9) {
      pipeline = 9;
      throw new IllegalArgumentException("Pipeline can not be greater than nine");
    }
    networkTable.getEntry("pipeline").setValue(pipeline);
  }

  /**
   * Returns current Pipeling of the Lime Light
   *
   * @return Pipelinge
   */
  public double getPipeline() {
    NetworkTableEntry pipeline = networkTable.getEntry("pipeline");
    double pipe = pipeline.getDouble(0.0);
    return pipe;
  }

  /**
   * Returns current Pipeling of the Lime Light
   *
   * @return Pipelinge
   */
  public Integer getPipelineInt() {
    NetworkTableEntry pipeline = networkTable.getEntry("pipeline");
    Integer pipe = (int) pipeline.getDouble(0.0);
    return pipe;
  }

  /**
   * stream Sets limelight’s streaming mode
   *
   * <p>kStandard - Side-by-side streams if a webcam is attached to Limelight kPiPMain - The
   * secondary camera stream is placed in the lower-right corner of the primary camera stream
   * kPiPSecondary - The primary camera stream is placed in the lower-right corner of the secondary
   * camera stream
   *
   * @param stream
   */
  public void setStream(StreamType stream) {
    networkTable.getEntry("stream").setValue(stream.getValue());
  }

  public StreamType getStream() {
    NetworkTableEntry stream = networkTable.getEntry("stream");
    double st = stream.getDouble(0.0);
    StreamType mode = StreamType.getByValue(st);
    return mode;
  }

  /**
   * snapshot Allows users to take snapshots during a match
   *
   * <p>kon - Stop taking snapshots koff - Take two snapshots per second
   *
   * @param snapshot
   */
  public void setSnapshot(Snapshot snapshot) {
    networkTable.getEntry("snapshot").setValue(snapshot.getValue());
  }

  public Snapshot getSnapshot() {
    NetworkTableEntry snapshot = networkTable.getEntry("snapshot");
    double snshot = snapshot.getDouble(0.0);
    Snapshot mode = Snapshot.getByValue(snshot);
    return mode;
  }

  public double calculateNorm(double x, double y) {

    setpoint = new Translation2d(x, y);
    robotPos =
        new Translation2d(
            getRobotPosition()[0], getRobotPosition()[1]); // MUST CHOOSE THE RIGHT ARRAY ELEMENT

    double norm = (setpoint.minus(robotPos)).getNorm();

    return norm;
  }

  public Rotation2d calculateAngle(double x, double y) {

    setpoint = new Translation2d(x, y);
    robotPos =
        new Translation2d(
            getRobotPosition()[0], getRobotPosition()[1]); // MUST CHOOSE THE RIGHT ARRAY ELEMENT

    Rotation2d angle = (setpoint.minus(robotPos)).getAngle();

    return angle;
  }

  public double[] getTagID() {
    return networkTable.getEntry("tid").getDoubleArray(new double[6]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
