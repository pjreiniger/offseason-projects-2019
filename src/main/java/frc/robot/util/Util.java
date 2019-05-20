package frc.robot.util;

/**
 * Class for Utility functions/variables that fall into no particular class 
 * and used in most places of the codebase.
 */
public class Util {

  private Util() {

  }

  public static double kElipson = 1e-6;

  public static boolean epsilonEquals(double a, double b, double elipson) {
    return (a - elipson <= b) && (a + elipson >= b);
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, kElipson);
  }

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
}