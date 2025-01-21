package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Transform3d;

public class CameraConfig {
  Transform3d cameraPose;
  Transform3d fiducialOffset;
  String name;

  public CameraConfig(String name, Transform3d cameraPose, Transform3d fiducialOffset) {
    this.cameraPose = cameraPose;
    this.fiducialOffset = fiducialOffset;
    this.name = name;
  }
}
