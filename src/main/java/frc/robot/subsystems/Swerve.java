package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Ports;

import frc.robot.lib.team254.geometry.Pose2d;
import frc.robot.lib.team254.geometry.Rotation2d;
import frc.robot.lib.team254.geometry.Translation2d;
import frc.robot.lib.vectors.VectorField;

import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

import frc.robot.subsystems.requests.Request;

import frc.robot.util.SwerveHeadingController;
import frc.robot.util.SwerveInverseKinematics;
import frc.robot.util.Util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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
    frontRight = new SwerveDriveModule(Ports.FR_RADIAL, Ports.FR_THROTTLE, 0, 
        Constants.kFrontRightEncoderStaringPos, Constants.kVehicleToModuleZero);
    frontLeft = new SwerveDriveModule(Ports.FR_RADIAL, Ports.FL_THROTTLE, 1, 
        Constants.kFrontLeftEncoderStartingPos, Constants.kVehicleToModuleOne);
    backRight = new SwerveDriveModule(Ports.BR_RADIAL, Ports.BR_THROTTLE, 2, 
        Constants.kBackRightEncoderStartingPos, Constants.kVehicleToModuleTwo);
    backLeft = new SwerveDriveModule(Ports.BL_RADIAL, Ports.BL_THROTTLE, 3, 
        Constants.kBackLeftEncoderStartingPos, Constants.kVehicleToModuleThree);

    modules = Arrays.asList(frontRight, frontLeft, backLeft, backRight);
    positionModules = Arrays.asList(frontRight, frontLeft, backLeft, backRight);

    modules.forEach((m) -> m.reverseRotationSensor(true));

    pigeon = Pigeon.getInstance();

    pose = new Pose2d();
    distanceTraveled = 0;

    lift = Lift.getInstance();
  }

  public void setCarpetDirection(boolean standardDirection) {
    modules.forEach((m) -> m.setCarpertDirection(standardDirection));
  }

  private Translation2d translationVector = new Translation2d();
  private double rotationalInput = 0;
  private Translation2d lastDriveVector = new Translation2d();
  private final Translation2d rotationalVector = Translation2d.identity();
  private double lowPowerScalar = 0.6;

  public void setLowPowerScalar(double scalar) {
    lowPowerScalar = scalar;
  }

  private double maxSpeedFactor = 1.0;

  public void setMaxSpeed(double max) {
    maxSpeedFactor = max;
  }

  private boolean robotCentric = false;

  // Going to places 254 wish they could go to
  VectorField vf;

  private SwerveInverseKinematics inverseKinematics = new SwerveInverseKinematics();

  public void setCenterOfRotation(Translation2d center) {
    inverseKinematics.setCenterOfRotation(center);
  }

  public enum ControlState {
    NEUTRAL, MANUAL, POSITION, VELOCITY, ROTATION, DISABLED,
    VECTORIZED, TRAJECTORY
  }

  private ControlState currentState = ControlState.NEUTRAL;

  public ControlState getState() {
    return currentState;
  }

  public void setState(ControlState state) {
    currentState = state;
  }

  public void sendInput(double x, double y, double rotate, boolean robotCentric, boolean lowPower) {
    Translation2d translationalInput = new Translation2d(x, y);
    double inputMagnitude = translationalInput.norm();

    double threshold = Math.toRadians(10.0);
    if (Math.abs(translationalInput.direction()
        .distance(translationalInput.direction().nearestPole())) < threshold) {
      translationalInput = translationalInput.direction()
          .nearestPole().toTranslation();
    }

    double deadband = 0.25;

    if (inputMagnitude < deadband) {
      translationalInput = new Translation2d();
      inputMagnitude = 0;
    }

    final double power = (lowPower) ? 1.75 : 1.5;
    Rotation2d direction = translationalInput.direction();
    double scaledMagnitude = Math.pow(inputMagnitude, power);
    translationalInput = new Translation2d(direction.cos() * scaledMagnitude, 
        direction.sin() * scaledMagnitude);
    rotate = (Math.abs(rotate) < deadband) ? 0 : rotate;
    rotate = Math.pow(Math.abs(rotate), 1.75) * Math.signum(rotate);

    translationalInput = translationalInput.scale(maxSpeedFactor);
    rotate *= maxSpeedFactor;

    translationVector = translationalInput;

    if (lowPower) {
      translationVector = translationVector.scale(lowPowerScalar);
      rotate *= lowPowerScalar;
    } else {
      rotate *= 0.8;
    }

    if (rotate != 0 && rotationalInput == 0) {
      headingController.disable();
    } else if (rotate == 0 && rotationalInput != 0) {
      headingController.tempDisable();
    }

    rotationalInput = rotate;

    if (translationalInput.norm() != 0) {
      if (currentState != ControlState.MANUAL) {
        setState(ControlState.MANUAL);
      }
    } else if (rotationalInput != 0) {
      if (currentState != ControlState.MANUAL && currentState != ControlState.TRAJECTORY) {
        setState(ControlState.MANUAL);
      }
    }

    if (inputMagnitude > 0.3) {
      lastDriveVector = new Translation2d(x, y);
    } else if (translationVector.x() == 0.0 && translationVector.y() == 0.0 && rotate != 0) {
      lastDriveVector = rotationalVector;
    }

    this.robotCentric = robotCentric;
  }

  public Rotation2d averagedDirection = Rotation2d.identity();
  
  public void resetAveragedDirection() {
    averagedDirection = pose.getRotation();
  }

  public void setAveragedDirection(double degrees) {
    averagedDirection = Rotation2d.fromDegrees(degrees);
  }

  public final double rotationDirectionThreshold = Math.toRadians(5.0);
  public final double rotationDivision = 1.0;

  public synchronized void updateControllerDirection(Translation2d input) {
    if (Util.epsilonEquals(input.norm(), 1.0, 0.1)) {
      Rotation2d direction = input.direction();
      double roundedDirection = Math.round(direction.getDegrees() / rotationDivision);
      averagedDirection = Rotation2d.fromDegrees(roundedDirection);
    }
  }

  public synchronized void rotate(double goalHeading) {
    if (translationVector.x() == 0 && translationVector.y() == 0) {
      rotateInPlace(goalHeading);
    } else {
      headingController.setStabilizationTarget(Util.placeIn0To360Scope(
          pose.getRotation().getUnboundedDegrees(), goalHeading));
    }
  }

  public void rotateInPlace(double goalHeading) {
    setState(ControlState.ROTATION);
    headingController.setStationaryTarget(Util.placeIn0To360Scope(
        pose.getRotation().getUnboundedDegrees(), goalHeading));
  }

  public void rotateInPlaceAbsolutely(double absoluteHeading) {
    setState(ControlState.ROTATION);
    headingController.setStationaryTarget(absoluteHeading);
  }

  public void setPathHeading(double goalHeading) {
    headingController.setSnapTarget(
        Util.placeIn0To360Scope(pose.getRotation().getUnboundedDegrees(), goalHeading));
  }

  public void setAbsolutePathHeading(double absoluteHeading) {
    headingController.setSnapTarget(absoluteHeading);
  }

  public void setPositionTarget(double directionDegrees, double magnitudeInches) {
    setState(ControlState.POSITION);
    modules.forEach(m -> m.setModuleAngle(directionDegrees));
    modules.forEach(m -> m.setDrivePositionTarget(magnitudeInches));
  }

  public void lockDrivePosition() {
    modules.forEach(m -> m.setDrivePositionTarget(0.0));
  }

  public void setVelocity(Rotation2d direction, double velocityInchesPerSecond) {
    setState(ControlState.VELOCITY);
    modules.forEach(m -> m.setModuleAngle(direction.getDegrees()));
    modules.forEach(m -> m.setVelocitySetpoint(velocityInchesPerSecond));
  }

  public void setDriveOutput(List<Translation2d> driveVectors) {
    for (int i = 0; i < modules.size(); i++) {
      if (Util.shouldReverse(driveVectors.get(i).direction().getDegrees(), 
          modules.get(i).getModuleAngle().getDegrees())) {
        modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
        modules.get(i).setDriveOpenLoop(driveVectors.get(i).norm());
      } else {
        modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
        modules.get(i).setDriveOpenLoop(driveVectors.get(i).norm());
      }
    }
  }

  public void setDriveOutput(List<Translation2d> driveVectors, double percentOutputOverdrive) {
    for (int i = 0; i < modules.size(); i++) {
      if (Util.shouldReverse(driveVectors.get(i).direction().getDegrees(), 
          modules.get(i).getModuleAngle().getDegrees())) {
        modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
        modules.get(i).setDriveOpenLoop(-percentOutputOverdrive);
      } else {
        modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
        modules.get(i).setDriveOpenLoop(percentOutputOverdrive);
      }
    }
  }

  public void setVelocityDriveOutput(List<Translation2d> driveVectors, double velocityOverdrive) {
    for (int i = 0; i < modules.size(); i++) {
      if (Util.shouldReverse(driveVectors.get(i).direction().getDegrees(), 
          modules.get(i).getModuleAngle().getDegrees())) {
        modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
        modules.get(i).setVelocitySetpoint(-velocityOverdrive);
      } else {
        modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
        modules.get(i).setVelocitySetpoint(velocityOverdrive);
      }
    }
  }

  public void setModuleAngles(List<Translation2d> driveVectors) {
    for (int i = 0; i < modules.size(); i++) {
      if (Util.shouldReverse(driveVectors.get(i).direction().getDegrees(), 
          modules.get(i).getModuleAngle().getDegrees())) {
        modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
      } else {
        modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
      }
    }
  }

  public void set10VoltRotationMode(boolean tenVolts) {
    modules.forEach(m -> m.set10VoltRotationMode(tenVolts));
  }

  public boolean positionOnTarget() {
    boolean onTarget = false;
    for (SwerveDriveModule m : modules) {
      onTarget |= m.drivePositionOnTarget();
    }
    return onTarget;
  }

  public boolean moduleAnglesOnTarget() {
    boolean onTarget = true;
    for (SwerveDriveModule m : modules) {
      onTarget &= m.angleOnTarget();
    }
    return onTarget;
  }

  // Vector Fields (EXPERIMENTAL, TO BE TESTED IN SIMS)

  public synchronized void setVectorField(VectorField vectorfield) {
    vf = vectorfield;
    setState(ControlState.VECTORIZED);
  }

  public synchronized void determineEvasiveWheels() {
    Translation2d here = lastDriveVector.rotateBy(pose.getRotation().inverse());
    List<Translation2d> wheels = Constants.kModulePositions;
    clockwiseCenter = wheels.get(0);
    counterClockwiseCenter = wheels.get(wheels.size() - 1);

    for (int i = 0; i < wheels.size() - 1; i++) {
      Translation2d cw = wheels.get(i);
      Translation2d ccw = wheels.get(i + 1);
      if (here.isWithinAngle(cw, ccw)) {
        clockwiseCenter = cw;
        counterClockwiseCenter = ccw;
      }
    }

  }

  public synchronized void updatePose(double timestamp) {
    double x = 0.0;
    double y = 0.0;
    Rotation2d heading = pigeon.getYaw();

    double averageDistance = 0.0;
    double[] distances = new double[modules.size()];
    for (SwerveDriveModule m : positionModules) {
      m.updatePose(heading);
      double distance = m.getEstimatedRobotPose().getTranslation()
          .translateBy(pose.getTranslation().inverse()).norm();
      distances[m.moduleID] = distance;
      averageDistance += distance;
    }

    averageDistance /= positionModules.size();

    int minDevianceIndex = 0;
    double minDeviance = 100.0;
    List<SwerveDriveModule> modulesToUse = new ArrayList<>();
    for (SwerveDriveModule m : positionModules) {
      double deviance = Math.abs(distances[m.moduleID] - averageDistance);
      if (deviance < minDeviance) {
        minDeviance = deviance;
        minDevianceIndex = m.moduleID;
      }
      if (deviance <= 0.01) {
        modulesToUse.add(m);
      }
    }

    if (modulesToUse.isEmpty()) {
      modulesToUse.add(modules.get(minDevianceIndex));
    }

    for (SwerveDriveModule m : modulesToUse) {
      x += m.getEstimatedRobotPose().getTranslation().x();
      y += m.getEstimatedRobotPose().getTranslation().y();
    }

    Pose2d updatedPose = new Pose2d(new Translation2d(x / modulesToUse.size(), 
        y / modulesToUse.size()), heading);
    double deltaPos = updatedPose.getTranslation().distance(pose.getTranslation());
    distanceTraveled += deltaPos;
    currentVel = deltaPos / (timestamp - lastUpdateTimestamp);
    pose = updatedPose;
    modules.forEach(m -> m.resetPose(pose));
  }

  public synchronized void alternatePoseUpdate() {
    double x = 0.0;
    double y = 0.0;
    Rotation2d heading = pigeon.getYaw();

    double[][] distances = new double[4][2];

    for (SwerveDriveModule m : modules) {
      m.updatePose(heading);
      double distance = m.getEstimatedRobotPose().getTranslation()
          .translateBy(pose.getTranslation().inverse()).norm();
      distances[m.moduleID][0] = m.moduleID;
      distances[m.moduleID][1] = distance;
    }

    Arrays.sort(distances, (double[] a, double[] b) -> Double.compare(a[1], b[1]));

    List<SwerveDriveModule> modulesToUse = new ArrayList<>();
    
    double firstDifference = distances[1][1] - distances[0][1];
    double secondDifference = distances[2][1] - distances[1][1];
    double thirdDifference = distances[3][1] - distances[2][1];
    if (secondDifference > (1.5 * firstDifference)) {
      modulesToUse.add(modules.get((int) distances[0][0]));
      modulesToUse.add(modules.get((int) distances[1][0]));
    } else if (thirdDifference > (1.5 * firstDifference)) {
      modulesToUse.add(modules.get((int) distances[0][0]));
      modulesToUse.add(modules.get((int) distances[1][0]));
      modulesToUse.add(modules.get((int) distances[2][0]));
    } else {
      modulesToUse.add(modules.get((int) distances[0][0]));
      modulesToUse.add(modules.get((int) distances[1][0]));
      modulesToUse.add(modules.get((int) distances[2][0]));
      modulesToUse.add(modules.get((int) distances[3][0]));
    }

    for (SwerveDriveModule m : modulesToUse) {
      x += m.getEstimatedRobotPose().getTranslation().x();
      y += m.getEstimatedRobotPose().getTranslation().y();
    }

    Pose2d updatedPose = new Pose2d(new Translation2d(x / modulesToUse.size(), 
        y / modulesToUse.size()), heading);
    double deltaPos = updatedPose.getTranslation().distance(pose.getTranslation());
    distanceTraveled += deltaPos;
    pose = updatedPose;
    modules.forEach(m -> m.resetPose(pose));
  }

  public synchronized void updateControlCycle(double timestamp) {
    double rotationCorrection = headingController.updateRotationCorrection(
          pose.getRotation().getUnboundedDegrees(), timestamp);

    switch (currentState) {
      case MANUAL:
        if (evading && evadingToggled) {
          determineEvasiveWheels();
          double sign = Math.signum(rotationalInput);
          if (sign == 1.0) {
            inverseKinematics.setCenterOfRotation(clockwiseCenter);
          } else if (sign == -1.0) {
            inverseKinematics.setCenterOfRotation(counterClockwiseCenter);
          }
          evadingToggled = false;
        } else if (evading) {
          double sign = Math.signum(rotationalInput);
          if (sign == 1.0) {
            inverseKinematics.setCenterOfRotation(clockwiseCenter);
          } else if (sign == -1.0) {
            inverseKinematics.setCenterOfRotation(counterClockwiseCenter);
          }
        } else if (evadingToggled) {
          inverseKinematics.setCenterOfRotation(Translation2d.identity());
          evadingToggled = false;
        }
        if (translationVector.equals(Translation2d.identity()) && rotationalInput == 0.0) {
          if (lastDriveVector.equals(rotationalVector)) {
            stop();
          } else {
            setDriveOutput(inverseKinematics.updateDriveVectors(lastDriveVector, 
                rotationCorrection, pose, robotCentric), 0.0);
          }
        } else {
          setDriveOutput(inverseKinematics.updateDriveVectors(translationVector, 
              (rotationalInput + rotationCorrection), pose, robotCentric), 0.0);
        }
        break;
      case POSITION:
        if (positionOnTarget()) {
          rotate(headingController.getTargetHeading());
        }
        break;
      case ROTATION:
        setDriveOutput(inverseKinematics.updateDriveVectors(new Translation2d(), 
            Util.deadBand(rotationCorrection, 0.1), pose, false));
        break;
      case VECTORIZED:
        Translation2d outputVectorV = vf.getVector(pose.getTranslation()).scale(0.25);
        setDriveOutput(inverseKinematics.updateDriveVectors(outputVectorV, rotationCorrection, 
            getPose(), false));
        break;
      case TRAJECTORY:
        // I dont even have auton done, why does this exist?!
        // Well not JUST autons, maybe macros so Casey can relax
        // At least programming will have its own version of "it worked in the cad"
        break;
      case VELOCITY:
        break;
      case NEUTRAL:
        stop();
        break;
      case DISABLED:
        break;
      default:
        break;
    }
  }
  
  private final Loop loop = new Loop() {

    @Override
    public void onStart(double timestamp) {
      synchronized (Swerve.this) {
        translationVector = new Translation2d();
        lastDriveVector = rotationalVector;
        rotationalInput = 0;
        headingController.tempDisable();
        stop();
        lastUpdateTimestamp = timestamp;
      }
    }

    @Override
    public void onLoop(double timestamp) {
      synchronized (Swerve.this) {
        if (modulesReady || (getState() != ControlState.TRAJECTORY)) {
          updatePose(timestamp);
        }
        updateControlCycle(timestamp);
        lastUpdateTimestamp = timestamp;
      }
    }

    @Override
    public void onStop(double timestamp) {
      synchronized (Swerve.this) {
        translationVector = new Translation2d();
        rotationalInput = 0;
        stop();
      }
    }
    
  };

  public Request openLoopRequest(Translation2d input, double rotation) {
    return new Request() {

      @Override
      public void act() {
        setState(ControlState.MANUAL);
        sendInput(input.x(), input.y(), rotation, false, false);  
      }
      
    };
  }

  public Request velocityRequest(Rotation2d direction, double magnitude) {
    return new Request() {

      @Override
      public void act() {
        setVelocity(direction, magnitude);
      }
      
    };
  }

  public void setNominalDriveOutput(double voltage) {
    modules.forEach(m -> m.setNominalOutput(voltage));
  }

  public void setMaxRotationSpeed() {
    double currentDriveSpeed = translationVector.norm() * Constants.kSwerveMaxSpeedInchesPerSecond;
    double newMaxRotationSpeed = Constants.kSwerveRotationMaxSpeed  
        / ((Constants.kSwerveRotationSpeedScalar * currentDriveSpeed) + 1.0);
    modules.forEach(m -> m.setMaxRotationSpeed(newMaxRotationSpeed));
  }

  @Override
  public void readPeriodicInputs() {
    modules.forEach(m -> m.readPeriodicInputs());
  }

  @Override
  public void writePeriodicOutputs() {
    modules.forEach(m -> m.writePeriodicOutputs());
  }

  @Override
  public void registerEnabledLooper(ILooper enabledLooper) {
    enabledLooper.register(loop);
  }

  @Override
  public void stop() {
    setState(ControlState.NEUTRAL);
    modules.forEach(m -> m.stop());
  }

  @Override
  public void outputTelemetery() {
    modules.forEach(m -> m.outputTelemetery());
    SmartDashboard.putNumberArray("Robot Pose", new double[]{pose.getTranslation().x(), 
        pose.getTranslation().y(), pose.getRotation().getUnboundedDegrees()});
    SmartDashboard.putString("Swerve State", currentState.toString());

    if (Constants.kDebuggingOutput) {
      // do something
    }
  }

}