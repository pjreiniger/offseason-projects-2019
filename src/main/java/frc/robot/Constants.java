package frc.robot;

import frc.robot.lib.team254.geometry.Pose2d;
import frc.robot.lib.team254.geometry.Rotation2d;
import frc.robot.lib.team254.geometry.Translation2d;
import java.util.Arrays;
import java.util.List;

public class Constants {
  // Miscellaneous
  public static final boolean kDebuggingOutput = false;
  public static final double kLooperDt = 0.02;
  public static double kElipson = 1e-6;
  public static final boolean kIsUsingTractionWheels = false;

  // Robot Dimensions (with bumpers i geuss)
  public static final double kRobotWidth = 0;
  public static final double kRobotLength = 0;
  public static final double kRobotHalfWidth = kRobotWidth / 2.0;
  public static final double kRobotHalfLength = kRobotLength / 2.0;

  // Swerve Physical Dimensions
  public static final double kWheelBaseLength = 10.0;
  public static final double kWheelBaseWidth = 10.0;
  public static final double kSwerveDiagonalDist = Math.hypot(kWheelBaseLength, kWheelBaseWidth);

  // Swerve
  public static final int kNumberOfModules = 4;
  public static final double kSwerveEncUnitsPerInch = 0.5;
  public static final double kSwerveRotationMaxSpeed = 0.5;
  public static final double kSwerveRotation10VoltMaxSpeed = 0.5;
  public static final double kSwerveDriveMaxSpeed = 0.5;
  public static final double kSwerveMaxSpeedInchesPerSecond = 12.5 * 12.0;
  public static final double kSwerveRotationSpeedScalar = 0.5;
  public static final double kSwerveDriveEncoderResolution = 0.5;
  public static final double[] kWheelScrubFactors = new double[]{1.0, 1.0, 1.0, 1.0};
  public static final double kXScrubFactor = 0.5;
  public static final double kYScrubFactor = 0.5;
  public static final boolean kSimulateReversedCarpet = false;
  public static final Pose2d kRobotLeftStartingPose = 
      new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));

  public static final int kFrontRightEncoderStaringPos = 0;
  public static final int kFrontLeftEncoderStartingPos = 0;
  public static final int kBackRightEncoderStartingPos = 0;
  public static final int kBackLeftEncoderStartingPos = 0;

  public static final Translation2d kVehicleToModuleZero = 
      new Translation2d(kWheelBaseLength / 2, kWheelBaseWidth / 2);
  public static final Translation2d kVehicleToModuleOne = 
      new Translation2d(-kWheelBaseLength / 2, kWheelBaseWidth / 2);
  public static final Translation2d kVehicleToModuleTwo = 
      new Translation2d(-kWheelBaseLength / 2, -kWheelBaseWidth / 2);
  public static final Translation2d kVehicleToModuleThree = 
      new Translation2d(kWheelBaseLength / 2, -kWheelBaseWidth / 2);

  public static final List<Translation2d> kModulePositions = 
      Arrays.asList(kVehicleToModuleZero, kVehicleToModuleOne, 
      kVehicleToModuleTwo, kVehicleToModuleThree);
      
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