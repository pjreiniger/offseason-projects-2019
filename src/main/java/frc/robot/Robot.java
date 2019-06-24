/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.io.Xbox;

import frc.robot.lib.team254.geometry.Translation2d;

import frc.robot.loops.Looper;

import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;

import frc.robot.util.CrashTracker;
import frc.robot.util.Util;
import java.util.Arrays;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Swerve swerve;

  private Superstructure s;
  private SubsystemManager subsystems;

  private Looper enabledLooper = new Looper();
  private Looper disabledLooper = new Looper();

  private DriverStation ds = DriverStation.getInstance();

  private Xbox driver;
  private boolean flickRotation = false;
  private boolean robotCentric = false;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    s = Superstructure.getInstance();
    swerve = Swerve.getInstance();
    subsystems = new SubsystemManager(Arrays.asList(swerve, s));

    driver = new Xbox(0);
    driver.setDeadband(0.2);

    subsystems.registerEnabledLoops(enabledLooper);
    subsystems.registerDisabledLooper(disabledLooper);

    swerve.zeroSensors();

    swerve.stop();
  }

  public void allPeriodic() {
    subsystems.outputToSmartDashboard();
    enabledLooper.outputToSmartDashboard();
  }

  public void teleopConfig() {
    swerve.setNominalDriveOutput(0.0);
    swerve.set10VoltRotationMode(false);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    try {
      disabledLooper.stop();
      enabledLooper.start();
      teleopConfig();
    } catch (Throwable t) {
      CrashTracker.logMarkerCrash(t);
      throw t;
    }
  }

  @Override
  public void teleopPeriodic() {
    try {
      driver.update();

      oneControllerMode();

      allPeriodic();
    } catch (Throwable t) {
      CrashTracker.logMarkerCrash(t);
      throw t;
    }
  }

  @Override
  public void disabledInit() {
    try {
      enabledLooper.stop();
      subsystems.stop();
      disabledLooper.start();
    } catch (Throwable t) {
      CrashTracker.logMarkerCrash(t);
      throw t;
    }
  }

  @Override
  public void disabledPeriodic() {
    try {
      allPeriodic();
    } catch (Throwable t) {
      CrashTracker.logMarkerCrash(t);
      throw t;
    }
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  /** 
   * Mode if one controller is detected (or just swerve).
  */
  public void oneControllerMode() {
    double swerveYInput = driver.getX(Hand.kLeft);
    double swerveXInput = -driver.getY(Hand.kLeft);
    double swerveRotationInput = (flickRotation ? 0.0 : driver.getX(Hand.kRight));

    swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, 
        robotCentric, driver.leftTrigger.isBeingPressed());
    
    if (flickRotation) {
      swerve.updateControllerDirection(new Translation2d(
          -driver.getY(Hand.kRight), driver.getX(Hand.kRight)));
      if (!Util.epsilonEquals(Util.placeIn0To360Scope(
           swerve.getTargetHeading(), swerve.averagedDirection.getDegrees()),
           swerve.getTargetHeading(), swerve.rotationDivision / 2.0)) {
        swerve.rotate(swerve.averagedDirection.getDegrees());
      }
    }

    if (driver.bButton.isBeingPressed()) {
      swerve.rotate(90);
    } else if (driver.aButton.isBeingPressed()) {
      swerve.rotate(180);
    } else if (driver.xButton.isBeingPressed()) {
      swerve.rotate(270);
    } else if (driver.leftBumper.shortReleased()) {
      swerve.rotate(-24);
    } else if (driver.leftBumper.longPressed()) {
      swerve.rotate(-151.0);
    } else if (driver.rightBumper.shortReleased()) {
      swerve.rotate(24);
    } else if (driver.rightBumper.longPressed()) {
      swerve.rotate(151.0);
    } else if (driver.POV0.isBeingPressed()) {
      swerve.rotate(0.0);
    }

    if (driver.backButton.shortReleased() || driver.backButton.longPressed()) {
      swerve.tempDisableHeadingController();
      swerve.zeroSensors(Constants.kRobotLeftStartingPose);
      swerve.resetAveragedDirection();
    } 

  }

}
