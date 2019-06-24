package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.lib.team254.geometry.Pose2d;
import frc.robot.lib.team254.geometry.Rotation2d;
import frc.robot.lib.team254.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

public class SwerveInverseKinematics {

  private List<Translation2d> moduleRelativePositions = Constants.kModulePositions;
  private List<Translation2d> moduleRotationDirections = updateRotationDirections();
  
  public SwerveInverseKinematics() {
    setCenterOfRotation(new Translation2d());
  }

  private List<Translation2d> updateRotationDirections() {
    List<Translation2d> directions = new ArrayList<>(Constants.kNumberOfModules);
    for (int i = 0; i < Constants.kNumberOfModules; i++) {
      directions.add(moduleRelativePositions.get(i).rotateBy(Rotation2d.fromDegrees(90)));
    }
    return directions;
  }

  /**
   * Set center of rotation for Swerve Modules to follow.
   * @param center Translatio2d of the center of rotation
   */
  public void setCenterOfRotation(Translation2d center) {
    List<Translation2d> positions = new ArrayList<>(Constants.kNumberOfModules);
    double maxMagnitude = 0.0;
    for (int i = 0; i < Constants.kNumberOfModules; i++) {
      Translation2d position = Constants.kModulePositions.get(i).translateBy(center);
      positions.add(position);
      double magnituide = position.norm();
      if (magnituide > maxMagnitude) {
        maxMagnitude = magnituide;
      }
    }

    for (int i = 0; i < Constants.kNumberOfModules; i++) {
      Translation2d position = positions.get(i);
      positions.set(i, position.scale(1.0 / maxMagnitude));
    }
    moduleRelativePositions = positions;
    moduleRotationDirections = updateRotationDirections();
  }

  /**
   * Update the drive vectors which the swerve modules will follow.
   * @param translationalVector Translational Vector the swerves will follow
   * @param rotationalMagnitude Magnitude of the rotational vector
   * @param robotPose Current Pose2d of the robot (Translation/Rotation)
   * @param robotCentric Is the swerve drive robot centric
   * @return
   */
  public List<Translation2d> updateDriveVectors(Translation2d translationalVector, 
      double rotationalMagnitude, Pose2d robotPose, boolean robotCentric) {
    SmartDashboard.putNumber("Vector Direction", translationalVector.direction().getDegrees());
    
    SmartDashboard.putNumber("Robot Velocity", translationalVector.norm());

    if (!robotCentric) {
      translationalVector = translationalVector.rotateBy(robotPose.getRotation().inverse());
    }
    List<Translation2d> driveVectors = new ArrayList<>(Constants.kNumberOfModules);
    for (int i = 0; i < Constants.kNumberOfModules; i++) {
      driveVectors.add(translationalVector.translateBy(
          moduleRotationDirections.get(i).scale(rotationalMagnitude)));
    }
    double maxMagnitude = 1.0;
    for (Translation2d t : driveVectors) {
      double magnituide = t.norm();
      if (magnituide > maxMagnitude) {
        maxMagnitude = magnituide;
      }
    }
    for (int i = 0; i < Constants.kNumberOfModules; i++) {
      Translation2d driveVector = driveVectors.get(i);
      driveVectors.set(i, driveVector.scale(1.0 / maxMagnitude));
    }
    return driveVectors;
  }

}