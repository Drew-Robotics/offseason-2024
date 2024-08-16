package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Collections;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private final Camera m_frontLeft, m_frontRight, m_backLeft, m_backRight;
  private final List<Camera> m_cameras;

  private static VisionSubsystem m_instance;

  public static VisionSubsystem getInstance() {
    if (m_instance == null)
      m_instance = new VisionSubsystem();
    return m_instance;
  }

  private VisionSubsystem() {
    m_frontLeft = new Camera(VisionConstants.CameraNames.kFrontLeft, VisionConstants.CameraTransforms.kFrontLeft);  
    m_frontRight = new Camera(VisionConstants.CameraNames.kFrontRight, VisionConstants.CameraTransforms.kFrontRight);
    m_backLeft = new Camera(VisionConstants.CameraNames.kBackLeft, VisionConstants.CameraTransforms.kBackLeft);
    m_backRight = new Camera(VisionConstants.CameraNames.kBackRight, VisionConstants.CameraTransforms.kBackRight);

    m_cameras = List.of(m_frontLeft, m_frontRight, m_backLeft, m_backRight);
  }
  public List<Optional<EstimatedRobotPose>> getCameraEstimatedPoses() {
      List<Optional<EstimatedRobotPose>> retVal = Collections.emptyList();
      for(Camera camera : m_cameras) {
          retVal.add(camera.getEstimatedGlobalPose());
      }
      return retVal;
  }

    public List<Optional<Matrix<N3, N1>>> getPoseStdDevs(List<Optional<EstimatedRobotPose>> poses) {
        List<Optional<Matrix<N3, N1>>> poseStdDevs = Collections.emptyList();

        for (int cameraIndex = 0; cameraIndex < m_cameras.size(); cameraIndex++) {
          Camera camera = m_cameras.get(cameraIndex);
          Pose2d pose = poses.get(cameraIndex).get().estimatedPose.toPose2d();

          if(poses.get(cameraIndex).isPresent())
            poseStdDevs.add(Optional.ofNullable(camera.getEstimationStdDevs(pose)));
          else
            poseStdDevs.add(Optional.empty());
        }
        return poseStdDevs;
    }

}