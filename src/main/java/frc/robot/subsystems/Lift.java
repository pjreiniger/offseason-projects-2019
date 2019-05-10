package frc.robot.subsystems;

import frc.robot.util.LazyTalonSRX;

public class Lift extends Subsystem {
  private static Lift instance = null;
  
  LazyTalonSRX master;
  LazyTalonSRX slave;
  
  /**
   * Returns an instance of Lift, mandates that only one version of Lift exists.
   */
  public static Lift getInstance() {
    if (instance == null) {
      instance = new Lift();
    }
    return instance;
  }

  @Override
  public void stop() {

  }

  @Override
  public void outputTelemetery() {

  }

}