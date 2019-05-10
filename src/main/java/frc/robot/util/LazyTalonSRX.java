package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * To avoid those annoying missed CAN Frames, and to avoid decimating the CAN Bus, So Chase can wire
 * without stress (Ɛ> Chase).
 */
public class LazyTalonSRX extends TalonSRX {
  protected ControlMode lastControlMode = null;
  protected double lastSet = Double.NaN;

  public LazyTalonSRX(int deviceNumber) {
    super(deviceNumber);
    super.configFactoryDefault();
  }

  public double getLastSet() {
    return lastSet;
  }

  @Override
  public void set(ControlMode mode, double value) {
    if (lastControlMode != mode || lastSet != value) {
      lastControlMode = mode;
      lastSet = value;
      super.set(mode, value);
    }
  }
}