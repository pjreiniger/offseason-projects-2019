package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import frc.robot.util.LazyTalonSRX;
import frc.robot.util.LazyVictorSPX;
import java.util.Arrays;
import java.util.List;


public class Drivetrain extends Subsystem {
  private static Drivetrain instance = null;

  LazyTalonSRX rightMaster;
  LazyVictorSPX rightSlaveA;
  LazyVictorSPX rightSlaveB;

  LazyTalonSRX leftMaster;
  LazyVictorSPX leftSlaveA;
  LazyVictorSPX leftSlaveB;

  List<BaseMotorController> motors;
  List<LazyVictorSPX> slaves;

  PeriodicIO periodicIO = new PeriodicIO();

  public enum ControlState {
    Neutral, OpenLoop, Position
  }

  private ControlState state = ControlState.Neutral;

  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }

  private Drivetrain() {
    motors = Arrays.asList(rightMaster, rightSlaveA, rightSlaveB, 
        leftMaster, leftSlaveA, leftSlaveB);
    slaves = Arrays.asList(rightSlaveA, rightSlaveB, leftSlaveA, leftSlaveB);

    for (BaseMotorController motor : motors) {
      motor.configVoltageCompSaturation(12.0, 10);
      motor.enableVoltageCompensation(true);
      motor.setNeutralMode(NeutralMode.Brake);
    }

    configForTeleop();
  }

  public void configForTeleop() {

  }



  @Override
  public void stop() {

  }

  @Override
  public void outputTelemetery() {

  }

  public static class PeriodicIO {
    public double controlInput = 0.0;

    public double demand;
  }
}