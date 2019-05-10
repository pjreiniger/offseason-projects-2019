package frc.robot.subsystems.requests;

/**
 * All actions that must be done by a subsystem before a request
 * can be completed.
 */

public abstract class Prerequisite {
  public abstract boolean met();
}