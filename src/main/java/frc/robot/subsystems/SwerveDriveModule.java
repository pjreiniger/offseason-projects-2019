package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.team254.geometry.Pose2d;
import frc.robot.lib.team254.geometry.Rotation2d;
import frc.robot.lib.team254.geometry.Translation2d;
import frc.robot.util.LazyTalonSRX;
import frc.robot.util.Util;

public class SwerveDriveModule extends Subsystem {
  LazyTalonSRX driveMotor;
  LazyTalonSRX rotationMotor;

  int moduleID;
  String name = "Module ";
  int rotationSetPoint = 0;
  double driveSetPoint = 0;
  int encoderOffset;
  int encoderReverseFactor = 1;
  boolean useDriveEncoder = true;
  boolean tenVoltRotationMode = false;
  private double previousEncDist = 0;
  private Translation2d position;
  private Translation2d startingPosition;
  private Pose2d estimatedRobotPose = new Pose2d();
  boolean standardCarpetDirection = true;

  public void setCarpertDirection(boolean direction) {
    standardCarpetDirection = direction;
  }

  PeriodicIO periodicIO = new PeriodicIO();

  /**
   * Constructor for SwerveDriveModule, yeah.
   * @param rotationSlot ID for rotation motor
   * @param driveSlot  ID for drive motor
   * @param moduleID ID for swerve module
   * @param encOffset Is the module not starting at 0!?
   * @param startingPose The translation from origin of Module
   */
  public SwerveDriveModule(int rotationSlot, int driveSlot, int moduleID, int encOffset,
      Translation2d startingPose) {
    name += moduleID + " ";
    rotationMotor = new LazyTalonSRX(rotationSlot);
    driveMotor = new LazyTalonSRX(driveSlot);
    this.moduleID = moduleID;
    this.encoderOffset = encOffset;
    previousEncDist = 0;
    position = startingPose;
    this.startingPosition = startingPose;
  }

  public synchronized void invertDriveMotor(boolean invert) {
    driveMotor.setInverted(invert);
  }

  public synchronized void invertRotationMotor(boolean invert) {
    rotationMotor.setInverted(invert);
  }

  public synchronized void reverseDriveSensor(boolean reverse) {
    driveMotor.setSensorPhase(reverse);
  }

  public synchronized void reverseRotationSensor(boolean reverse) {
    rotationMotor.setSensorPhase(reverse);
  }

  public synchronized void setNominalOutput(double voltage) {
    driveMotor.configNominalOutputForward(voltage / 12.0, 10);
    driveMotor.configNominalOutputReverse(-voltage / 12.0, 10);
  }

  public synchronized void setMaxRotationSpeed(double maxSpeed) {
    rotationMotor.configMotionCruiseVelocity((int) maxSpeed, 10);
  }

  public synchronized void disableDriveSensor() {
    useDriveEncoder = false;
  }

  private void configureMotors() {
    rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    rotationMotor.setSensorPhase(true);
    rotationMotor.setInverted(false);
    rotationMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 10);
    rotationMotor.setNeutralMode(NeutralMode.Brake);
    rotationMotor.configVoltageCompSaturation(7.0, 10);
    rotationMotor.enableVoltageCompensation(true);
    rotationMotor.configAllowableClosedloopError(0, 0, 10);
    rotationMotor.configMotionAcceleration((int) (Constants.kSwerveRotationMaxSpeed * 12.5), 10);
    rotationMotor.configMotionCruiseVelocity((int)(Constants.kSwerveRotationMaxSpeed), 10);
    rotationMotor.selectProfileSlot(0, 0);

    // Normal Use
    rotationMotor.config_kP(0, 1.0, 10);
    rotationMotor.config_kI(0, 1.0, 10);
    rotationMotor.config_kD(0, 10, 10);
    rotationMotor.config_kF(0, 1023.0 / Constants.kSwerveRotationMaxSpeed, 10);

    // Auto Slot (why is this neccesary, S I M G A N G)
    rotationMotor.config_kP(1, 1.0, 10);
    rotationMotor.config_kI(1, 1.0, 10);
    rotationMotor.config_kD(1, 10, 10);
    rotationMotor.config_kF(1, 1023.0 / Constants.kSwerveRotation10VoltMaxSpeed, 10);

    rotationMotor.set(ControlMode.MotionMagic, rotationMotor.getSelectedSensorPosition(0));
    if (!isRotationMotorSensorConnected()) {
      DriverStation.reportError(name + "rotation encoder deid", false);
      hasEmergency = true;
    }

    driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    driveMotor.setSelectedSensorPosition(0, 0, 10);
    driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 10);
    driveMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 10);
    driveMotor.configVelocityMeasurementWindow(32, 10);
    driveMotor.configNominalOutputForward(1.5 / 12.0, 10);
    driveMotor.configNominalOutputReverse(-1.5 / 12.0, 10);
    driveMotor.configVoltageCompSaturation(12.0, 10);
    driveMotor.enableVoltageCompensation(true);
    driveMotor.configOpenloopRamp(0.25, 10);
    driveMotor.configClosedloopRamp(0.0);
    driveMotor.configAllowableClosedloopError(0, 0, 10);
    driveMotor.setSensorPhase(true);
    driveMotor.setInverted(true);
    driveMotor.setNeutralMode(NeutralMode.Brake);

    driveMotor.configMotionCruiseVelocity((int) (Constants.kSwerveDriveMaxSpeed * 0.6), 10);
    driveMotor.configMotionAcceleration((int) (Constants.kSwerveDriveMaxSpeed), 10);

    // MotionMagic Slot
    driveMotor.selectProfileSlot(0, 0);
    driveMotor.config_kP(0, 1.0, 10);
    driveMotor.config_kI(0, 1.0, 10);
    driveMotor.config_kD(0, 10, 10);
    driveMotor.config_kF(0, 1023.0 / Constants.kSwerveDriveMaxSpeed, 10);

    // Velocity Type Slot 
    driveMotor.config_kP(1, 0.1, 10);
    driveMotor.config_kI(1, 0.0, 10);
    driveMotor.config_kD(1, 10, 10);
    driveMotor.config_kF(1, 1023.0 / Constants.kSwerveDriveMaxSpeed * 0.9, 10);
    
    if (!isDriveMotorSensorConnected()) {
      DriverStation.reportError(name + "drive encoder deid", false);
      hasEmergency = true;
    }
  }

  private boolean isRotationMotorSensorConnected() {
    int pulseWidthPeriod = rotationMotor.getSensorCollection().getPulseWidthRiseToRiseUs();
    return pulseWidthPeriod != 0;
  }

  private boolean isDriveMotorSensorConnected() {
    int pulseWidthPeriod = driveMotor.getSensorCollection().getPulseWidthRiseToRiseUs();
    return pulseWidthPeriod != 0;
  }

  private double getRawAngle() {
    return encUnitsToInches(periodicIO.rotationPos);
  }

  public Rotation2d getModuleAngle() {
    return Rotation2d.fromDegrees(getRawAngle() - encUnitsToDegrees(encoderOffset));
  }

  public Rotation2d getFieldCentricAngle(Rotation2d robotHeading) {
    Rotation2d normalizedAngle = getModuleAngle();
    return normalizedAngle.rotateBy(robotHeading);
  }

  /**
   * Set the angle of the drive module.
   * @param goalAngle Angle to get to.
   */
  public void setModuleAngle(double goalAngle) {
    double newAngle = Util.placeIn0To360Scope(getRawAngle(), 
        goalAngle + encUnitsToDegrees(encoderOffset));
    int setpoint = degreesToEncUnits(newAngle);
    periodicIO.rotationControlMode = ControlMode.MotionMagic;
    periodicIO.rotationDemand = setpoint;
  }

  public boolean angleOnTarget() {
    double error = encUnitsToDegrees(Math.abs(periodicIO.rotationDemand - periodicIO.rotationPos));
    return error < 4.5;
  }

  /**
   * Supply rotation motor with 10 volts (quicker rotations).
   * @param tenVolts Enable or Disable ten volt mode.
   */
  public void set10VoltRotationMode(boolean tenVolts) {
    if (tenVolts && !tenVoltRotationMode) {
      rotationMotor.selectProfileSlot(1, 0);
      rotationMotor.configVoltageCompSaturation(10.0, 10);
      tenVoltRotationMode = true;
    } else if (!tenVolts && tenVoltRotationMode) {
      rotationMotor.selectProfileSlot(0, 0);
      rotationMotor.configVoltageCompSaturation(7.0, 10);
      tenVoltRotationMode = false;
    }
  }

  public void setRotationOpenLoop(double power) {
    periodicIO.rotationControlMode = ControlMode.PercentOutput;
    periodicIO.rotationDemand = power;
  }

  public void setDriveOpenLoop(double velocity) {
    periodicIO.driveControlMode = ControlMode.PercentOutput;
    periodicIO.driveDemand = velocity;
  }

  /**
   * Set distance for drive module to travel.
   * @param deltaDistanceInches distance for drive module to travel.
   */
  public void setDrivePositionTarget(double deltaDistanceInches) {
    driveMotor.selectProfileSlot(0, 0);
    periodicIO.driveControlMode = ControlMode.MotionMagic;
    periodicIO.driveDemand = periodicIO.drivePos + inchesToEncUnits(deltaDistanceInches);
  }

  /**
   * Checks whether the drive has reached its MotioMagic target.
   * @return boolean if whether the destination was reached.
   */
  public boolean drivePositionOnTarget() {
    if (driveMotor.getControlMode() == ControlMode.MotionMagic) {
      return encUnitsToInches((int)Math.abs(periodicIO.driveDemand - periodicIO.drivePos)) < 2.0;
    }
    return false;
  }

  /**
   * Set velocity for drive module to achieve.
   * @param inchesPerSecond Velocity for drive module to get to.
   */
  public void setVelocitySetpoint(double inchesPerSecond) {
    driveMotor.selectProfileSlot(1, 0);
    periodicIO.driveControlMode = ControlMode.Velocity;
    periodicIO.driveDemand = inchesPerSecondToEncVelocity(inchesPerSecond);
  }

  private double getDriveDistanceInches() {
    return encUnitsToInches(periodicIO.drivePos);
  }

  public double encUnitsToInches(double encUnits) {
    return encUnits / Constants.kSwerveEncUnitsPerInch;
  }

  public int inchesToEncUnits(double inches) {
    return (int) (inches * Constants.kSwerveEncUnitsPerInch);
  }

  public double encVelocityToInchesPerSecond(double encUnitsPer100ms) {
    return encUnitsToInches(encUnitsPer100ms) * 10;
  }

  public int inchesPerSecondToEncVelocity(double inchesPerSecond) {
    return (int) (inchesToEncUnits(inchesPerSecond / 10.0));
  }

  public int degreesToEncUnits(double degrees) {
    return (int) (degrees / 360.0 * Constants.kSwerveDriveEncoderResolution);
  }

  public double encUnitsToDegrees(double encUnits) {
    return encUnits / Constants.kSwerveDriveEncoderResolution * 360.0;
  }

  public Translation2d getPosition() {
    return position;
  }

  public Pose2d getEstimatedRobotPose() {
    return estimatedRobotPose;
  }

  /**
   * Update the Pose of the Robot.
   * @param robotHeading Current Heading of the Robot.
   */
  public synchronized void updatePose(Rotation2d robotHeading) {
    double currentEncDist = getDriveDistanceInches();
    double deltaEncDist = (currentEncDist - previousEncDist) 
        * Constants.kWheelScrubFactors[moduleID];
    Rotation2d currentWheelAngle = getFieldCentricAngle(robotHeading);
    Translation2d deltaPosition = new Translation2d(currentWheelAngle.cos() * deltaEncDist,
        currentWheelAngle.sin() * deltaEncDist);
    double xscrubFactor = Constants.kXScrubFactor;
    double yscrubFactor = Constants.kYScrubFactor;
    if (Constants.kSimulateReversedCarpet) {
      if (Util.epsilonEquals(Math.signum(deltaPosition.x()), 1.0)) {
        if (standardCarpetDirection) {
          xscrubFactor = 1.0 / Constants.kXScrubFactor;
        } else {
          xscrubFactor = 1.0;
        }
      } else {
        if (standardCarpetDirection) {
          xscrubFactor = Math.pow(Constants.kXScrubFactor, 2);
        } else {
          // Do Nothing I guess
        }
      }
      if (Util.epsilonEquals(Math.signum(deltaPosition.y()), 1.0)) {
        if (standardCarpetDirection) {
          yscrubFactor = 1.0 / Constants.kYScrubFactor;
        } else {
          yscrubFactor = 1.0;
        }
      } else {
        if (standardCarpetDirection) {
          yscrubFactor = Math.pow(Constants.kYScrubFactor, 2);
        } else {
          // Do Nothing I guess
        }
      }
    } else {
      if (Util.epsilonEquals(Math.signum(deltaPosition.x()), 1.0)) {
        if (standardCarpetDirection) {
          xscrubFactor = 1.0;
        } else {
          // Do Nothing I guess
        }
      } else {
        if (standardCarpetDirection) {
          // Do Nothing I guess
        } else {
          xscrubFactor = 1.0;
        }
      }
      if (Util.epsilonEquals(Math.signum(deltaPosition.y()), 1.0)) {
        if (standardCarpetDirection) {
          yscrubFactor = 1.0;
        } else {
          // Do Nothing I guess
        }
      } else {
        if (standardCarpetDirection) {
          // Do Nothing I guess  
        } else {
          yscrubFactor = 1.0;
        }
      }
    }
    
    deltaPosition = new Translation2d(deltaPosition.x() * xscrubFactor, 
        deltaPosition.y() * yscrubFactor);
    Translation2d updatedPosition = position.translateBy(deltaPosition);
    Pose2d staticWheelPose = new Pose2d(updatedPosition, robotHeading);
    Pose2d robotPose = staticWheelPose.transformBy(
        Pose2d.fromTranslation(startingPosition).inverse());
    position = updatedPosition;
    estimatedRobotPose = robotPose;
    previousEncDist = currentEncDist;
  }

  /**
   * Reset the Pose of the Robot.
   * @param robotPose Pose to reset to.
   */
  public synchronized void resetPose(Pose2d robotPose) {
    Translation2d modulePosition = robotPose.transformBy(
        Pose2d.fromTranslation(startingPosition)).getTranslation();
    position = modulePosition;
  }

  public synchronized void resetPose() {
    position = startingPosition;
  }

  public synchronized void resetLastEncoderReading() {
    previousEncDist = getDriveDistanceInches();
  }

  @Override
  public synchronized void readPeriodicInputs() {
    periodicIO.rotationPos = rotationMotor.getSelectedSensorPosition(0);
    if (useDriveEncoder) {
      periodicIO.drivePos = driveMotor.getSelectedSensorPosition(0);
    }
    if (Constants.kDebuggingOutput) {
      periodicIO.velocity = driveMotor.getSelectedSensorVelocity();
    }
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    rotationMotor.set(periodicIO.rotationControlMode, periodicIO.rotationDemand);
    driveMotor.set(periodicIO.driveControlMode, periodicIO.driveDemand);
  }

  @Override
  public void stop() {
    setDriveOpenLoop(0.0);
  }

  public synchronized void disable() {
    setDriveOpenLoop(0.0);
    setRotationOpenLoop(0.0);
  }

  public synchronized void resetRotationToAbsolute() {
    rotationMotor.setSelectedSensorPosition(encoderReverseFactor 
        * (rotationMotor.getSensorCollection().getPulseWidthPosition() - encoderOffset), 0, 10);
  }

  @Override
  public synchronized void zeroSensors() {
    zeroSensors(new Pose2d());
  }

  /**
   * Zero all Sensors on Robot with Pose.
   * @param robotPose Pose to reset to.
   */
  public synchronized void zeroSensors(Pose2d robotPose) {
    resetPose(robotPose);
    estimatedRobotPose = robotPose;
    previousEncDist = getDriveDistanceInches();
  }

  @Override
  public void outputTelemetery() {
    SmartDashboard.putNumber(name + "Angle", getModuleAngle().getDegrees());
    SmartDashboard.putNumber(name + "Inches Driven", getDriveDistanceInches());
    if (Constants.kDebuggingOutput) {
      SmartDashboard.putNumber(name + "Pulse Width", rotationMotor.getSelectedSensorPosition(0));
      SmartDashboard.putNumber(name + "Drive Voltage", periodicIO.driveVoltage);
      SmartDashboard.putNumber(name + "Rotation Voltage", rotationMotor.getMotorOutputVoltage());
      SmartDashboard.putNumber(name + "Velocity", 
          encVelocityToInchesPerSecond(periodicIO.velocity));
      if (rotationMotor.getControlMode() == ControlMode.MotionMagic) {
        SmartDashboard.putNumber(name + "Error", 
            encUnitsToDegrees(rotationMotor.getClosedLoopError()));
      }
      // SmartDashboard.putNumber(name + "X", position.x());
      // SmartDashboard.putNumber(name + "Y", position.y());
      SmartDashboard.putNumber(name + "Drive Current", driveMotor.getOutputCurrent());
      SmartDashboard.putNumber(name + "Rotation Speed", rotationMotor.getSelectedSensorVelocity());
    }
  }

  public static class PeriodicIO {
    public int rotationPos = 0;
    public int drivePos = 0;
    public int velocity = 0;
    public double driveVoltage = 0.0;

    public ControlMode rotationControlMode = ControlMode.PercentOutput;
    public ControlMode driveControlMode = ControlMode.PercentOutput;
    public double rotationDemand;
    public double driveDemand;
  }
}