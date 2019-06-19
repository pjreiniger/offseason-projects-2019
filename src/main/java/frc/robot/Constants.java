package frc.robot;

public class Constants {
  // Miscellaneous
  public static final boolean kDebuggingOutput = false;
  public static final double kLooperDt = 20;

  // Swerve
  public static final double kSwerveEncUnitsPerInch = 0.5;
  public static final double kSwerveRotationMaxSpeed = 0.5;
  public static final double kSwerveRotation10VoltMaxSpeed = 0.5;
  public static final double kSwerveDriveMaxSpeed = 0.5;
  public static final double kSwerveDriveEncoderResolution = 0.5;
  public static final double[] kWheelScrubFactors = new double[4];
  public static final double kXScrubFactor = 0.5;
  public static final double kYScrubFactor = 0.5;
  public static final boolean kSimulateReversedCarpet = false;

  // Elevator Constants
  public static final double kMaxElevatorTeleopSpeed = 0.1;
  public static final double kMaxElevatorHeight = 10;
  public static final double kMinElevatorHeight = 5;
  public static final double kMaxInitialElevatorHeight = 10;
  public static final double kMinInitialElevatorHeight = 10;
  public static final double kEncTicksPerInch = 10;
  public static final double kElevatorEncoderStartingPosition = 0.5;
  public static final double kElevatorHeightTolerance = 0.01;
  public static final double kElevatorMaxHighGear = 10;
}