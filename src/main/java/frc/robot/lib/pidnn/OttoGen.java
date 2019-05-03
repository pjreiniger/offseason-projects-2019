package frc.robot.lib.pidnn;

import java.util.function.Supplier;

public class OttoGen {
  private double[][] w1;
  private double[][] w2;
  private double previousPos;
  private double prevX22;
  private double prevX32;
  private int n;
  private Supplier<Double> currentPos;
  private Supplier<Double> targetPos;

  /**
   * Constructor for OttoGen v1.
   * @param currentPos Function that supplies subsystems current position
   * @param targetPos Function that supplies subsystems target position
   */
  public OttoGen(Supplier<Double> currentPos, Supplier<Double> targetPos) {
    this.currentPos = currentPos;
    this.targetPos = targetPos;
    this.previousPos = 0;
    this.prevX22 = 0;
    this.prevX32 = 0;
    this.n = 400;
    w1 = NP.random(2, 3);
    w2 = NP.random(3, 1);
  }

  /**
   * Trains the PIDNN instance instantiated.
   */
  public void train() {
    for (int i = 0; i < n; i++) {
      double[][] input = {{targetPos.get(), previousPos}};
      double[][] u2 = NP.dot(input, w1);
      double[][] x2 = transform(u2);
      double[][] u3 = NP.dot(x2, w2);
      double x3 = u3[0][0];
    }

  }

  public void execute() {

  }

  /**
   * Transforms Matrix m based on transformation functions of a PIDNN.
   * @param m Matrix to be transposed
   * @return Transformed Matrix
   */
  public double[][] transform(double[][] m) {
    for (int i = 0; i < m[0].length; i++) {
      switch (i) {
        case 0:
          break;
        case 1:
          m[0][i] = prevX22 + m[0][i];
          if (m[0][i] > 60000) {
            m[0][i] = 60000;
          } else if (m[0][i] < -60000) {
            m[0][i] = -60000;
          }
          prevX22 = m[0][i];
          break;
        case 2:
          m[0][i] = m[0][i] - prevX32;
          prevX32 = m[0][i];
          break;
        default:
          System.out.println("INVALID MATRIX");
          break;
      }
    }

    return m;
  }
}
