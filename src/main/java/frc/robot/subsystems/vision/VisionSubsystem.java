package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.StructArrayPublisher;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Subsystem;

public class VisionSubsystem extends Subsystem {
  private final Camera m_frontLeft, m_frontRight, m_backLeft, m_backRight;
  private final List<Camera> m_cameras;

  AprilTag[] m_tags = Constants.VisionConstants.AprilTags.kTags.toArray(AprilTag[]::new);;
  Pose3d[] m_testPoeses;

  private VisionSubsystemLogger m_logger;
  private class VisionSubsystemLogger {
    private final StructArrayPublisher<AprilTag> fieldTagsPublisher = m_table.getStructArrayTopic("FieldTags", new AprilTagStruct()).publish();
    // private final StructArrayPublisher<AprilTag> seenTagsPublisher = m_table.getStructArrayTopic("SeenTags", new AprilTagStruct()).publish();
  }

  private static VisionSubsystem m_instance;
  public static VisionSubsystem getInstance() {
    if (m_instance == null)
      m_instance = new VisionSubsystem();
    return m_instance;
  }

  private VisionSubsystem() {
    super(VisionSubsystem.class.getSimpleName());

    m_frontLeft = new Camera(VisionConstants.CameraNames.kFrontLeft, VisionConstants.CameraTransforms.kFrontLeft);  
    m_frontRight = new Camera(VisionConstants.CameraNames.kFrontRight, VisionConstants.CameraTransforms.kFrontRight);
    m_backLeft = new Camera(VisionConstants.CameraNames.kBackLeft, VisionConstants.CameraTransforms.kBackLeft);
    m_backRight = new Camera(VisionConstants.CameraNames.kBackRight, VisionConstants.CameraTransforms.kBackRight);

    m_cameras = List.of(m_frontLeft, m_frontRight, m_backLeft, m_backRight);

    m_logger = new VisionSubsystemLogger();
  }

  /* ----- OVERRIDES ----- */

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void dashboardInit() {}
  @Override
  public void dashboardPeriodic() {}
  @Override
  public void publishInit() {}
  @Override
  public void publishPeriodic() {
    // m_logger.seenTagsPublisher.accept(); // todo : finish this
    m_logger.fieldTagsPublisher.accept(m_tags);
  }

  /* ----- VISION ------ */

  public List<Optional<EstimatedRobotPose>> getCameraEstimatedPoses() {
    ArrayList<Optional<EstimatedRobotPose>> retVal = new ArrayList<Optional<EstimatedRobotPose>>();

    for(Camera camera : m_cameras) {
      retVal.add(camera.getEstimatedGlobalPose());
    }
    return retVal;
  }

    public List<Optional<Matrix<N3, N1>>> getPoseStdDevs(List<Optional<EstimatedRobotPose>> poses) {
      ArrayList<Optional<Matrix<N3, N1>>> poseStdDevs = new ArrayList<Optional<Matrix<N3, N1>>>();

      for (int cameraIndex = 0; cameraIndex < m_cameras.size(); cameraIndex++) {
        Camera camera = m_cameras.get(cameraIndex);

        Optional<EstimatedRobotPose> poseOptional = poses.get(cameraIndex);

        if(poseOptional.isPresent())
          poseStdDevs.add(Optional.ofNullable(camera.getEstimationStdDevs(
            poseOptional.get().estimatedPose.toPose2d()
          )));
        else
          poseStdDevs.add(Optional.empty());
      }
      return poseStdDevs;
    }

}