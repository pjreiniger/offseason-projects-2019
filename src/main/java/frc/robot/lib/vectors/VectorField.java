package frc.robot.lib.vectors;

import frc.robot.lib.team254.geometry.Translation2d;

public abstract class VectorField implements IVectorField {
  public abstract Translation2d getVector(Translation2d here);

  private VectorField() {

  }
  
  /**
   * Add components of another Vector Field to this VectorField.
   * 
   * @param vf Vector Field to be added
   * @return Vector field with new translated vectors
   */
  public VectorField add(VectorField vf) {
    VectorField temp = this;

    return new VectorField() {

      @Override
      public Translation2d getVector(Translation2d here) {
        return temp.getVector(here).translateBy(vf.getVector(here)).normalize();
      }

    };
  }

  /**
   * Return the inverse of the vector in the VectorField.
   * 
   * @return VectorField containing inverse vector
   */
  public VectorField inverse() {
    VectorField temp = this;

    return new VectorField() {

      @Override
      public Translation2d getVector(Translation2d here) {
        return temp.getVector(here).inverse();
      }

    };
  }

  /**
   * Scale down Vector in VectorField by some amount.
   * 
   * @param s Scaling Factor
   * @return Scaled Vector Field
   */
  public VectorField scale(double s) {
    VectorField temp = this;

    return new VectorField() {

      @Override
      public Translation2d getVector(Translation2d here) {
        return temp.getVector(here).scale(s);
      }

    };
  }



}
