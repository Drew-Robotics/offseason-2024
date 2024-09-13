package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer.subsystems;

public class GoToPosCommand extends Command{

  private final List<Pose2d> m_targetPoses;
  private final Rotation2d m_finalRot;

  private Command m_pathCommand;

  public GoToPosCommand(Pose2d targetPos, Rotation2d targetRot) {
    m_targetPoses = List.of(targetPos);
    m_finalRot = targetRot;

    addRequirements(subsystems.drive);
  }

  public GoToPosCommand(List<Pose2d> targetPoses, Rotation2d finalRot) {
    m_targetPoses = targetPoses;
    m_finalRot = finalRot;

    addRequirements(subsystems.drive);
  }

  @Override
  public void initialize() {
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(m_targetPoses);

    PathPlannerPath path = new PathPlannerPath(
      bezierPoints, 
      new PathConstraints(
        DriveConstants.Constraints.kTranslationalVelocity.in(Units.MetersPerSecond),
        DriveConstants.Constraints.kTranslationalAcceleration.in(Units.MetersPerSecondPerSecond),
        DriveConstants.Constraints.kRotationalVelocity.in(Units.RadiansPerSecond),
        DriveConstants.Constraints.kRotationalAcceleration.in(Units.RadiansPerSecond.per(Units.Second))
      ), 
      new GoalEndState(0, m_finalRot)
    );

    m_pathCommand = subsystems.drive.getPathCommand(path);

    // will stop the drive command because of conflict
    CommandScheduler.getInstance().schedule(m_pathCommand);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().cancel(m_pathCommand);
  }

  @Override
  public boolean isFinished() {
    // will be ended manually
    return false;
  }
}
