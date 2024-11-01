// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.notePipeline;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NotePipelineConstants.MotorSpeeds;
import frc.robot.RobotContainer.subsystems;

public class EjectNoteCommand extends Command {
  /** Creates a new EjectNoteCommand. */
  public EjectNoteCommand() {
    addRequirements(subsystems.feeder, subsystems.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystems.feeder.set(MotorSpeeds.ShooterEject.feeder * -1);
    subsystems.intake.set(MotorSpeeds.ShooterEject.intake * -1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystems.feeder.set(0);
    subsystems.intake.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
