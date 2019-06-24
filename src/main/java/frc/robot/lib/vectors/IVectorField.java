package frc.robot.lib.vectors;

import frc.robot.lib.team254.geometry.Translation2d;

public interface IVectorField {
  public abstract Translation2d getVector(Translation2d here);
}