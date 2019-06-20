package frc.robot.util;

import frc.robot.Constants;

/**
 * Class for Utility functions/variables that fall into no particular class 
 * and used in most places of the codebase.
 */
public class Util {

  private Util() {

  }

  public static boolean epsilonEquals(double a, double b, double elipson) {
    return (a - elipson <= b) && (a + elipson >= b);
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, Constants.kElipson);
  }

  /**
   * Bind a value to a specified scope.
   * @param lowerScope The minimum value of the scope
   * @param upperScope Number to be placed within scope
   * @return Scoped value
   */
  public static double boundToScope(double lowerScope, double upperScope, double argument) {
    double stepSize = upperScope - lowerScope;
    while (argument >= upperScope) {
      argument -= stepSize;
    }
    while (argument < lowerScope) {
      argument += stepSize;
    }
    return argument;
  }

  /**
   * The most basic way to bind an Angle within a 0-360 degree scope.
   * @param angle Angle to be placed in scope.
   * @return Bounded Angle
   */
  public static double boundAngle0to360Degrees(double angle) {
    while (angle >= 360.0) {
      angle -= 360.0;
    }
    while (angle < 0.0) {
      angle += 360.0;
    }
    return angle;
  }

  /**
   * The most basic way to bind an Angle within a -180-180 degree scope.
   * @param angle Angle to be placed in scope.
   * @return Bounded Angle
   */
  public static double boundAngleNeg180To180Degrees(double angle) {
    while (angle < -180.0) {
      angle += 360.0;
    }
    while (angle >= 180.0) {
      angle -= 360.0;
    }
    return angle;
  }

  /**
   * A much better way to bind an Angle in case it must be relative to another reference.
   * @param scopeReference  Reference for scope to be based upon.
   * @param newAngle Angle to be placed in 0-360 degree scope relative to reference.
   * @return Angle placed within 0-360 degree scope based on reference
   */
  public static double placeIn0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      // This should never even happen, but just in case
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference < 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }

  public static double deadBand(double val, double deadband) {
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }
}