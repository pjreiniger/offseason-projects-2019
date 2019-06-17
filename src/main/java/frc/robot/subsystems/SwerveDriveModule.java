package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;
import frc.robot.lib.team254.geometry.Pose2d;
import frc.robot.lib.team254.geometry.Translation2d;
import frc.robot.util.LazyTalonSRX;

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
  private Pose2d robotPose = new Pose2d();
  boolean standardCarpetDirection = true;

  public void setCarpertDirection(boolean direction) {
    standardCarpetDirection = direction;
  }

  PeriodicIO periodicIO = new PeriodicIO();

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

  public synchronized void invertDriveMotor() {

  }

  private double getRawAngle() {
    return encUnitsToInches(periodicIO.rotationPos);
  }

  public double encUnitsToInches(double encUnits) {
    return encUnits/Constants.kSwerveEncUnitsPerInch;
  }

  @Override
  public void stop() {

  }

  @Override
  public void outputTelemetery() {

  }

  public static class PeriodicIO {
    public int rotationPos = 0;
    public int drivePos = 0;
    public int velocity = 0;
    public double driveVoltage = 0.0;

    public ControlMode rotationControlMode = ControlMode.PercentOutput;
    public ControlMode driveControlMode = ControlMode.PercentOutput;
    public int rotationDemand;
    public int driveDemand;
  }
}