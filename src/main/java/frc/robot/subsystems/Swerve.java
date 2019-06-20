package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import frc.robot.Constants;
import frc.robot.lib.team254.geometry.Pose2d;
import frc.robot.lib.team254.geometry.Translation2d;
import frc.robot.util.SwerveHeadingController;

public class Swerve extends Subsystem {

  private static Swerve instance = null;

  /**
   * Returns an instance of Swerve.
   * @return instance of Swerve class
   */
  public static Swerve getInstance() {
    if (instance == null) {
      instance = new Swerve();
    }
    return instance;
  }

  public SwerveDriveModule frontRight;
  public SwerveDriveModule frontLeft;
  public SwerveDriveModule backLeft;
  public SwerveDriveModule backRight;

  List<SwerveDriveModule> modules;
  List<SwerveDriveModule> positionModules;

  Lift lift;

  Translation2d clockwiseCenter = new Translation2d();
  Translation2d counterClockwiseCenter = new Translation2d();
  boolean evading = false;
  boolean evadingToggled = false;

  public void toggleEvade() {
    evading = !evading;
    evadingToggled = true;
  }

  Pigeon pigeon;
  SwerveHeadingController headingController = new SwerveHeadingController();

  public void tempDisableHeadingController() {
    headingController.tempDisable();
  }

  public double getTargetHeading() {
    return headingController.getTargetHeading();
  }

  Pose2d pose;
  double distanceTraveled;
  double currentVel = 0;
  double lastUpdateTimestamp = 0;
  
  public Pose2d getPose() {
    return pose;
  }

  boolean modulesReady = false;
  boolean alwaysConfigureModules = false;
  boolean moduleConfigRequested = false;

  public void requireModuleConfiguration() {
    modulesReady = false;
  }

  public void alwaysConfigureModules() {
    alwaysConfigureModules = true;
  }

  Pose2d startingPose = Constants.kRobotLeftStartingPose;
  
  public void setStartingPose(Pose2d newPose) {
    startingPose = newPose;
  }

  private Swerve() {
    frontRight = new SwerveDriveModule(rotationSlot, driveSlot, moduleID, encOffset, startingPose);
    frontLeft = new SwerveDriveModule(rotationSlot, driveSlot, moduleID, encOffset, startingPose);
    backRight = new SwerveDriveModule(rotationSlot, driveSlot, moduleID, encOffset, startingPose);
    backLeft = new SwerveDriveModule(rotationSlot, driveSlot, moduleID, encOffset, startingPose);

    modules = Arrays.asList(frontRight, frontLeft, backLeft, backRight);
    positionModules = Arrays.asList(frontRight, frontLeft, backLeft, backRight);

    modules.forEach((m) -> m.reverseRotationSensor(true));

    pigeon = Pigeon.getInstance();

    pose = new Pose2d();
    distanceTraveled = 0;

    lift = Lift.getInstance();
  }


  @Override
  public void stop() {

  }

  @Override
  public void outputTelemetery() {

  }

}