package frc.robot.util;

/**
 * Implements a PID Control Loop.
 * 
 * <p>Does all PIDF computation simultaneously, important for systems such as swerve.
 * Must be calculated on a seperate thread.
 */
public class SynchronousPIDF {
  private double kp;
  private double ki;
  private double kd;
  private double kf;

  private double maxOutput = 1.0;
  private double minOutput = -1.0;

  private double maxInput = 0.0;
  private double minInput = 0.0;

  private boolean continous = false; // Absolute Encoder Gang
  private double prevError = 0.0;
  private double totalError = 0.0;

  private double setpoint = 0.0;
  private double error = 0.0;
  private double result = 0.0;
  private double last_input = Double.NaN;
  private double deadband = 0.0; // Error too small to matter, act like its 0



  public SynchronousPIDF() {

  }

  /**
   * Create a Synchronous PIDF Object with P, I, and D.
   * @param Kp Proportional Constant
   * @param Ki Integral Constant
   * @param Kd Derivative Constant
   */
  public SynchronousPIDF(double Kp, double Ki, double Kd) {
    kp = Kp;
    ki = Ki;
    kd = Kd;
    kf = 0;
  }

  /**
   * Create a synchronous PIDF object with P, I, D, and F.
   * @param Kp Proportional Constant
   * @param Ki Integral Constant
   * @param Kd Derivative Constant
   * @param Kf FeedForward Contant
   */
  public SynchronousPIDF(double Kp, double Ki, double Kd, double Kf) {
    kp = Kp;
    ki = Ki;
    kd = Kd;
    kf = Kf;
  }

  /**
   * Reads input, calculates and returns output.
   * Should be called periodically in a thread.
   * @param input Signal Input
   * @param dt time passed since previous call
   * @return
   */
  public double calculate(double input, double dt) {
    if (dt < 1e-6) {
      dt = 1e-6;
    }
    last_input = input;
    error = setpoint - input;
    if (continous) {
      if (Math.abs(error) > (maxInput - minInput) / 2) {
        if (error > 0) {
          error = error - maxInput + minInput;
        } else {
          error = error + maxInput - minInput;
        }
      }
    }

    if ((error * kp < maxOutput) && (error * kp > minOutput)) {
      totalError += error * dt;
    } else {
      totalError = 0;
    }

    double proportionalError = Math.abs(error) < deadband ? 0 : error;

    result = (kp * proportionalError 
        + ki * totalError + kd * (error - prevError) / dt + kf * setpoint);

    prevError = error;

    if (result > maxOutput) {
      result = maxOutput;
    } else if (result < minOutput) {
      result = minOutput;
    }

    return result;
  }

  /**
   * Set PID Controller Gains. Set proportional, intergal, and dervivative constants.
   * @param p Proportional Constant
   * @param i Integral Constant
   * @param d Derivative Constant
   */
  public void setPID(double p, double i, double d) {
    kp = p;
    ki = i;
    kd = d;
  }

  /**
   * Set PID Controller Gains. 
   * Set proportional, intergal, dervivative, and feedforward constants.
   * @param p Proportional Constant
   * @param i Integral Constant
   * @param d Derivative Constant
   * @param f Feedforward Constant
   */
  public void setPID(double p, double i, double d, double f) {
    kp = p;
    ki = i;
    kd = d;
    kf = f;
  }

  public double getP() {
    return kp;
  }

  public double getI() {
    return ki;
  }

  public double getD() {
    return kd;
  }

  public double getF() {
    return kf;
  }

  /**
   *  Returns the current PID result based on constraints.
   * @return the latest calculated output
   */
  public double get() {
    return result;
  }

  /**
   * Set the PID controller to consider the input to be continuous, Rather then using the max and
   * min in as constraints, it considers them to be the same point and automatically calculates the
   * shortest route to the setpoint.
   * 
   * @param cont Set to true turns on continuous, false turns off continuous
   */
  public void setContinuous(boolean cont) {
    continous = cont;
  }

  /**
   * Set the PID controller to consider the input to be continuous, Rather then using the max and
   * min in as constraints, it considers them to be the same point and automatically calculates the
   * shortest route to the setpoint.
   */
  public void setContinuous() {
    this.setContinuous(true);
  }

  public void setDeadband(double db) {
    deadband = db;
  }

  /**
   * Sets maximum and minimum values from input.
   * @param minimumInput The minimum value expected from input
   * @param maximumInput The maximum value expected from input
   */
  public void setInputRange(double minimumInput, double maximumInput) {
    if (minimumInput > maximumInput) {
      System.out.println("broke");
    }
    minInput = minimumInput;
    maxInput = maximumInput;
    setSetpoint(setpoint);
  }

  /**
   *  Sets maximum and minimum output to write.
   * @param minimumOutput The minimum output to write to output
   * @param maximumOutput The maximum output to write to output
   */
  public void setOutputRange(double minimumOutput, double maximumOutput) {
    if (minimumOutput > maximumOutput) {
      System.out.println("broke");
    }
    minOutput = minimumOutput;
    maxOutput = maximumOutput;
  }

  /**
   * Setpoint for PID Controller.
   * @param point The desired setpoint
   */
  public void setSetpoint(double point) {
    if (maxInput > minInput) {
      if (setpoint > maxInput) {
        setpoint = maxInput;
      } else if (setpoint < minInput) {
        setpoint = minInput;
      } else {
        setpoint = point;
      }
    } else {
      setpoint = point;
    }
  }

  public double getSetpoint() {
    return setpoint;
  }

  public double getError() {
    return error;
  }

  public boolean onTarget(double tolerance) {
    return last_input != Double.NaN && Math.abs(last_input - setpoint) < tolerance;
  }

  /**
   * Reset all internals.
   */
  public void reset() {
    last_input = Double.NaN;
    prevError = 0;
    totalError = 0;
    result = 0;
    setpoint = 0;
  }

  public void resetIntegrator() {
    totalError = 0;
  }

  /**
   * Returns the current PID Constansts.
   * @return String of all three states
   */
  public String getState() {
    String lstate = "";

    lstate += "Kp: " + kp + "\n";
    lstate += "Ki: " + ki + "\n";
    lstate += "Kd: " + kd + "\n";
    return lstate;
  }

  public String getType() {
    return "Synchronous PIDController";
  }
}