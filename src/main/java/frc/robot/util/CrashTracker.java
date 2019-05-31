package frc.robot.util;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.UUID;

public class CrashTracker {

  private static final UUID RUN_UUID_INSTANCE = UUID.randomUUID();

  public static void logRobotStartup() {
    logMarker("robot startup");
  }

  public static void robotInit() {
    logMarker("robot init");
  }

  public static void teleopInit() {
    logMarker("teleop init");
  }

  public static void autoInit() {
    logMarker("auto init");
  }

  public static void disabledInit() {
    logMarker("disabled init");
  }

  private static void logMarker(String mark) {
    logMarker(mark, null);
  }

  public static void logMarkerCrash(Throwable exception) {
    logMarker("exception", exception);
  }

  private static void logMarker(String mark, Throwable exception) {
    try (PrintWriter io = new PrintWriter(new FileWriter("/home/lvuser/crash_tracker.txt", true))) {
      io.print(RUN_UUID_INSTANCE.toString());
      io.print(", ");
      io.print(mark);
      io.print(", ");
      io.print(new Date().toString());

      if (exception != null) {
        io.print(", ");
        exception.printStackTrace(io);
      }

      io.println();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }
}