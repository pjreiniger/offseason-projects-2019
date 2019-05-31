package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.lib.team254.geometry.Pose2d;
import frc.robot.lib.team254.geometry.Translation2d;
import frc.robot.util.LazyTalonSRX;

public class SwerveDriveModule {
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