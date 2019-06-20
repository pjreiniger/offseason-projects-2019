package frc.robot;

import frc.robot.lib.team254.geometry.Pose2d;
import frc.robot.lib.team254.geometry.Rotation2d;

public class Constants {
  // Miscellaneous
  public static final boolean kDebuggingOutput = false;
  public static final double kLooperDt = 0.02;
  public static double kElipson = 1e-6;
  public static final boolean kIsUsingTractionWheels = false;


  // Swerve
  public static final double kSwerveEncUnitsPerInch = 0.5;
  public static final double kSwerveRotationMaxSpeed = 0.5;
  public static final double kSwerveRotation10VoltMaxSpeed = 0.5;
  public static final double kSwerveDriveMaxSpeed = 0.5;
  public static final double kSwerveDriveEncoderResolution = 0.5;
  public static final double[] kWheelScrubFactors = new double[]{1.0, 1.0, 1.0, 1.0};
  public static final double kXScrubFactor = 0.5;
  public static final double kYScrubFactor = 0.5;
  public static final boolean kSimulateReversedCarpet = false;
  public static final Pose2d kRobotLeftStartingPose = 
      new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));

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