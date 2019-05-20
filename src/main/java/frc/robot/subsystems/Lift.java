package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.requests.Prerequisite;
import frc.robot.subsystems.requests.Request;
import frc.robot.util.LazyTalonSRX;
import frc.robot.util.LazyVictorSPX;
import frc.robot.util.Util;
import java.util.Arrays;
import java.util.List;

public class Lift extends Subsystem {
  private static Lift instance = null;
  
  LazyTalonSRX liftRight;
  LazyVictorSPX liftLeft;
  List<BaseMotorController> motors;
  List<LazyVictorSPX> slaves;

  private double targetHeight = 0.0;
  private boolean configForAscent = true;
  private boolean limitsEnabled = false;

  double manualSpeed = Constants.kMaxElevatorTeleopSpeed;

  PeriodicIO periodicIO = new PeriodicIO();

  public enum ControlState {
    OpenLoop, Neutral, Position, Locked
  }

  private ControlState state = ControlState.Neutral;

  public static Lift getInstance() {
    if (instance == null) {
      instance = new Lift();
    }
    return instance;
  }

  public double getTargetHeight() {
    return targetHeight;
  }

  public ControlState getState() {
    return state;
  }

  public void setState(ControlState newState) {
    state = newState;
  }

  public void setManualSpeed(double speed) {
    manualSpeed = speed;
  }

  private Lift() {
    liftRight = new LazyTalonSRX(Ports.ELEVATOR_TALON);
    liftLeft = new LazyVictorSPX(Ports.ELEVATOR_VICTOR);

    motors = Arrays.asList(liftRight, liftLeft);
    slaves = Arrays.asList(liftLeft);

    slaves.forEach(s -> s.set(ControlMode.Follower, Ports.ELEVATOR_TALON));

    for (BaseMotorController motor : motors) {
      motor.configVoltageCompSaturation(12.0, 10);
      motor.enableVoltageCompensation(true);
      motor.setNeutralMode(NeutralMode.Brake);
    }
    
    liftRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    liftRight.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
    liftRight.configNominalOutputForward(0);
    liftRight.configNominalOutputReverse(0);
    liftRight.configPeakOutputForward(1);
    liftRight.configPeakOutputReverse(-1);
    liftRight.configForwardSoftLimitEnable(true, 10);
    liftRight.configReverseSoftLimitEnable(true, 10);

    configForAscent();
  }

  private void configForAscent() {
    manualSpeed = Constants.kMaxElevatorTeleopSpeed;

    liftRight.config_kF(0, 1023.0 / Constants.kElevatorMaxHighGear, 10);
    liftRight.config_kP(0, 2.5, 10);
    liftRight.config_kI(0, 0, 10);
    liftRight.config_kD(0, 25, 10);

    liftRight.config_kF(1, 1023.0 / Constants.kElevatorMaxHighGear, 10);
    liftRight.config_kP(1, 3.5, 10);
    liftRight.config_kI(1, 0, 10);
    liftRight.config_kD(1, 35, 10);

    liftRight.configMotionCruiseVelocity((int) (Constants.kElevatorMaxHighGear * 1.0), 10);
    liftRight.configMotionAcceleration((int) (Constants.kElevatorMaxHighGear * 3.0), 10);
    liftRight.configMotionSCurveStrength(0);

    configForAscent = true;
  }

  private void configForDescent() {
    liftRight.configMotionSCurveStrength(4);

    configForAscent = false;
  }

  public void configForTeleopSpeed() {
    configForAscent();
  }

  public void enableLimits(boolean enable) {
    liftRight.overrideSoftLimitsEnable(enable);
    limitsEnabled = enable;
  }

  private void setOpenLoop(double output) {
    setState(ControlState.OpenLoop);
    periodicIO.demand = output * manualSpeed;
  }

  public boolean isOpenLoop() {
    return getState() == ControlState.OpenLoop;
  }

  public synchronized void setTargetHeight(double heightFeet) {
    setState(ControlState.Position);
    if (heightFeet > Constants.kMaxElevatorHeight) {
      heightFeet = Constants.kMaxElevatorHeight;
    } else if (heightFeet < Constants.kMinElevatorHeight) {
      heightFeet = Constants.kMinElevatorHeight;
    }
    if (isSensorConnected()) {
      if (heightFeet > getHeight()) {
        liftRight.selectProfileSlot(0, 0);
        configForAscent();
      } else if (heightFeet < getHeight()) {
        liftRight.selectProfileSlot(1, 0);
        configForDescent();
      }
      targetHeight = heightFeet;
      periodicIO.demand = elevatorHeightToEncTicks(heightFeet);
      onTarget = false;
      startTime = Timer.getFPGATimestamp();
    } else {
      DriverStation.reportError("Elevator Encoder Dead", false);
      stop();
    }

  }

  public synchronized void lockHeight() {
    setState(ControlState.Locked);
    if (isSensorConnected()) {
      targetHeight = getHeight();
      periodicIO.demand = periodicIO.pos;
    } else {
      DriverStation.reportError("Elevator Encoder Dead", false);
      stop();
    }
  }

  public Request openLoopRequest(double output) {
    return new Request() {
    
      @Override
      public void act() {
        setOpenLoop(output);
      }
    };
  }

  public Request heightRequest(double height) {
    return new Request() {
    
      @Override
      public void act() {
        setTargetHeight(height);
      }

      @Override
      public boolean isFinished() {
        return hasReachedTargetHeight() || isOpenLoop();
      }
    };
  }

  public Request lockHeightRequest() {
    return new Request() {
    
      @Override
      public void act() {
        lockHeight();
      }
    };
  }

  public Prerequisite heightRequisite(double height, boolean above) {

    return new Prerequisite() {
    
      @Override
      public boolean met() {
        return Util.epsilonEquals(Math.signum(height - getHeight()), above ? 1.0 : -1.0);
      }
    };
  }

  boolean onTarget = false;
  double startTime = 0.0;

  public boolean hasReachedTargetHeight() {
    if (liftRight.getControlMode() == ControlMode.MotionMagic) {
      if (Math.abs(targetHeight - getHeight()) <= Constants.kElevatorHeightTolerance) {
        if (!onTarget) {
          System.out.println("Elevator done in: " + (Timer.getFPGATimestamp() - startTime));
          onTarget = true;
        }
        return true;
      }
    }
    return false;
  }

  @Override
  public void stop() {
    setOpenLoop(0.0);
  }

  @Override
  public void outputTelemetery() {
    SmartDashboard.putNumber("Elevator Height", getHeight());

    if (Constants.kDebuggingOutput) {
      SmartDashboard.putNumber("Elevator 1 Voltage", periodicIO.vol);
      SmartDashboard.putNumber("Elevator 2 Voltage", liftLeft.getMotorOutputVoltage());
      SmartDashboard.putNumber("Elevator 1 Current", periodicIO.cur);
      SmartDashboard.putNumber("Elevator Pulse Width Position", 
          liftRight.getSensorCollection().getPulseWidthPosition());
      SmartDashboard.putNumber("Elevator Encoder", periodicIO.pos);
      SmartDashboard.putNumber("Elevator Velocity", periodicIO.vel);
      SmartDashboard.putNumber("Elevator Error", liftRight.getClosedLoopError(0));
      if (liftRight.getControlMode() == ControlMode.MotionMagic) {
        SmartDashboard.putNumber("Elevator Setpoint", liftRight.getClosedLoopTarget());
      }
    }
  }

  private double encTicksToInches(double encTicks) {
    return encTicks / Constants.kEncTicksPerInch;
  }

  private int inchesToEncTicks(double inches) {
    return (int) (inches * Constants.kEncTicksPerInch);
  }

  public double getHeight() {
    return encTicksToElevatorHeight(periodicIO.pos);
  }

  public double encTicksToElevatorHeight(double encTicks) {
    return encTicksToInches(encTicks - Constants.kElevatorEncoderStartingPosition);
  }

  public double elevatorHeightToEncTicks(double elevatorHeight) {
    return Constants.kElevatorEncoderStartingPosition + inchesToEncTicks(elevatorHeight);
  }

  public boolean isSensorConnected() {
    int pulseWidthPeriod = liftRight.getSensorCollection().getPulseWidthRiseToRiseUs();
    boolean connected = pulseWidthPeriod != 0;
    if (!connected) {
      hasEmergency = true;
    }
    return connected;
  }

  public void resetToAbsolutePosition() {
    int position = (int) Util.boundToScope(0, 4096, 
        liftRight.getSensorCollection().getPulseWidthPosition());
    if (encTicksToElevatorHeight(position) > Constants.kMaxInitialElevatorHeight) {
      position -= 4096;
    } else if (encTicksToElevatorHeight(position) < Constants.kMinInitialElevatorHeight) {
      position += 4096;
    }
    double height = encTicksToElevatorHeight(position);
    if (height > Constants.kMaxInitialElevatorHeight  
        || height < Constants.kMinInitialElevatorHeight) {
      System.out.println("Elevator done goofed up");
      hasEmergency = true;
    }
    liftRight.setSelectedSensorPosition(position, 0, 10);
  }

  @Override
  public void readPeriodicInputs() {
    periodicIO.pos = liftRight.getSelectedSensorPosition(0);

    if (Constants.kDebuggingOutput) {
      periodicIO.vel = liftRight.getSelectedSensorVelocity(0);
      periodicIO.vol = liftRight.getMotorOutputVoltage();
      periodicIO.cur = liftRight.getOutputCurrent();
    }    
  }

  @Override
  public void zeroSensors() {
    resetToAbsolutePosition();
  }

  @Override
  public void writePeriodicOutputs() {
    if (getState() == ControlState.Position || getState() == ControlState.Locked) {
      liftRight.set(ControlMode.MotionMagic, periodicIO.demand);
    } else {
      liftRight.set(ControlMode.PercentOutput, periodicIO.demand);
    }
  }

  public static class PeriodicIO {
    public int pos = 0;
    public double vel = 0.0;
    public double vol = 0.0;
    public double cur = 0.0;

    public double demand;
  }

}