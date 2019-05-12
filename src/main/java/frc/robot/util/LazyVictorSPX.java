package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * To avoid those annoying missed CAN Frames, and to avoid decimating the CAN Bus, So Chase can wire
 * without stress (Ɛ> Chase).
 */
public class LazyVictorSPX extends VictorSPX {
  protected ControlMode lastControlMode = null;
  protected double lastSet = Double.NaN;

  public LazyVictorSPX(int deviceNumber) {
    super(deviceNumber);
    super.configFactoryDefault();
  }

  public double getLastSet() {
    return lastSet;
  }

  @Override
  public void set(ControlMode mode, double value) {
    if (mode != lastControlMode || value != lastSet) {
      lastControlMode = mode;
      lastSet = value;
      super.set(mode, value);
    }
  }
}