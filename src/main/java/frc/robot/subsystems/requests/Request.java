package frc.robot.subsystems.requests;

import java.util.ArrayList;
import java.util.List;

/**
 * Defines the structure of a command sent to a subsystem, or known as a Request.
 * This will aid in pinpointing exact commands that cause a fault in a subsystem
 * along with organizing the definition of the behavior of a subsystem.
 */
public abstract class Request {
  public abstract void act();

  public boolean isFinished() {
    return true;
  }

  public List<Prerequisite> prerequisites = new ArrayList<>();

  /**
   * Add a List of Prequisites that need to be done before acting upon a Request.   
  */
  public void withPrequisites(List<Prerequisite> prereqs) {
    for (Prerequisite req : prereqs) {
      prerequisites.add(req);
    }
  }

  /**
   * Add a Prequisite to the list of Prequisites that must be completed before a Request.
   */

  public void withPrequisites(Prerequisite prereq) {
    prerequisites.add(prereq);
  }

  /**
   * Checks if all prerequisites have been made before acting upon a Request.
   * @return Prerequisites met
   */
  public boolean allowed() {
    boolean reqsMet = true;
    for (Prerequisite req : prerequisites) {
      reqsMet &= req.met();
    }
    return reqsMet;
  }
}