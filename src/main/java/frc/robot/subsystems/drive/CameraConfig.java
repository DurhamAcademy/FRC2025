package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class CameraConfig {
    Transform3d cameraPose;
    Translation3d fiducialOffset;
    String name;

    public CameraConfig(String name, Transform3d cameraPose, Translation3d fiducialOffset) {
        this.cameraPose = cameraPose;
        this.fiducialOffset = fiducialOffset;
        this.name = name;
    }
}
