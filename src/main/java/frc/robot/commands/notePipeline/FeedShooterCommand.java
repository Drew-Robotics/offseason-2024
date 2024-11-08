// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.notePipeline;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NotePipelineConstants.MotorSpeeds;
import frc.robot.RobotContainer.subsystems;

public class FeedShooterCommand extends Command {
  /** Creates a new ShooterCommand. */
  public FeedShooterCommand() {
    addRequirements(subsystems.feeder);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    subsystems.feeder.set(MotorSpeeds.shooterFeed); // Feeds the note into the reving shooter
  }

  @Override
  public void end(boolean interrupted) {
      subsystems.feeder.stop();
  }
}
