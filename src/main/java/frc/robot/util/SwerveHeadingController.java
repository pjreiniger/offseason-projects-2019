package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class SwerveHeadingController {
  private double targetHeading;
  private double disabledTimestamp;
  private double lastUpdateTimestamp;
  private final double disableTimeLength = 0.2;
  private SynchronousPIDF stabilizationPID;
  private SynchronousPIDF snapPID;
  private SynchronousPIDF stationaryPID;

  public enum State {
    OFF, STABILIZE, SNAP, TEMPORARYDISABLE, STATIONARY
  }

  private State currentState = State.OFF;

  public State getState() {
    return currentState;
  }

  private void setState(State newState) {
    currentState = newState;
  }

  /**
   * Heading Controller for Swerve Base.
   */
  public SwerveHeadingController() {
    if (Constants.kIsUsingTractionWheels) {
      stabilizationPID = new SynchronousPIDF(0.005, 0, 0.005, 0.0);
      snapPID = new SynchronousPIDF(0.015, 0.0, 0.0, 0.0);
      stationaryPID = new SynchronousPIDF(0.02, 0.0, 0.002, 0.0);
    } else {
      stabilizationPID = new SynchronousPIDF(0.005, 0.0, 0.005, 0.0);
      snapPID = new SynchronousPIDF(0.015, 0.0, 0.0, 0.0);
      stationaryPID = new SynchronousPIDF(0.01, 0.0, 0.002, 0.0);
    }

    targetHeading = 0;
    lastUpdateTimestamp = Timer.getFPGATimestamp();
  }

  public void setStabilizationTarget(double angle) {
    targetHeading = angle;
    setState(State.STABILIZE);
  }

  public void setSnapTarget(double angle) {
    targetHeading = angle;
    setState(State.SNAP);
  }

  public void setStationaryTarget(double angle) {
    targetHeading = angle;
    setState(State.STATIONARY);
  }

  public void disable() {
    setState(State.OFF);
  }

  public void tempDisable() {
    setState(State.TEMPORARYDISABLE);
    disabledTimestamp = Timer.getFPGATimestamp();
  }

  public double getTargetHeading() {
    return targetHeading;
  }

  /**
   * Calculate the correction to get a target rotation.
   * @param heading current heading of rotation
   * @param timestamp time when correction is called
   * @return rotational correction
   */
  public double updateRotationCorrection(double heading, double timestamp) {
    double correction = 0;
    double error = heading - targetHeading;
    double dt = timestamp - lastUpdateTimestamp;

    switch (currentState) {
      case OFF:
        break;
      case TEMPORARYDISABLE:
        targetHeading = heading;
        if (timestamp - disabledTimestamp >= disableTimeLength) {
          setState(State.STABILIZE);
        }
        break;
      case STABILIZE:
        correction = stabilizationPID.calculate(error, dt);
        break;
      case SNAP:
        correction = snapPID.calculate(error, dt);
        break;
      case STATIONARY:
        correction = stationaryPID.calculate(error, dt);
        break;
      default:
        System.out.println("BAD SWERVE HEADING STATE");
        break;
    }

    lastUpdateTimestamp = timestamp;
    return correction;
  }
}