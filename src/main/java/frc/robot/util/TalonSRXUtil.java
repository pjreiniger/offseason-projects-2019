package frc.robot.util;

import com.ctre.phoenix.ErrorCode;
import edu.wpi.first.wpilibj.DriverStation;

public class TalonSRXUtil {

  /**
   * Actually be able to pinpoint exact Error Code and do more proper debugging.
   */
  public static void reportError(ErrorCode code, String message) {
    if (code != ErrorCode.OK) {
      DriverStation.reportError(message + code, false);
    }
  }
}