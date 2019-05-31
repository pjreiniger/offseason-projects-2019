package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;
import frc.robot.lib.team254.geometry.Rotation2d;
import frc.robot.util.LazyTalonSRX;

public class Pigeon {
  private static Pigeon instance = null;

  private PigeonIMU pigeon;

  public static Pigeon getInstance() {
    if (instance == null) {
      instance = new Pigeon();
    }
    return instance;
  }

  private Pigeon() {
    try {
      pigeon = new PigeonIMU(new LazyTalonSRX(Ports.GYRO));
    } catch (Exception e) {
      System.out.println(e);
    }
  }

  public boolean isGood() {
    return (pigeon.getState() == PigeonState.Ready) ? true : false;
  }

  public Rotation2d getYaw() {
    PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
    SmartDashboard.putNumber("Pigeon Heading", -pigeon.getFusedHeading());
    return Rotation2d.fromDegrees(-pigeon.getFusedHeading(fusionStatus));
  }

  public double getPitch() {
    double[] ypr = new double[3];
    pigeon.getYawPitchRoll(ypr);
    return ypr[1];
  }

  public double getRoll() {
    double[] ypr = new double[3];
    pigeon.getYawPitchRoll(ypr);
    return ypr[2];
  }

  public double[] getYPR() {
    double[] ypr = new double[3];
    pigeon.getYawPitchRoll(ypr);
    return ypr;
  }

  public void setAngle(double angle) {
    pigeon.setFusedHeading(-angle * 64.0, 10);
    pigeon.setYaw(-angle, 10);
    System.out.println("Pigeon Angle set to: " + angle);
  }

  public void outputToSmartDashboard() {
    SmartDashboard.putString("Pigeon Status", pigeon.getState().toString());
  }
}