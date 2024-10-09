// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.subsystems;

public class NotePipelineCommand extends Command {
  private DoubleSupplier m_mps;

  /** Creates a new NotePipelineCommand. */
  public NotePipelineCommand(DoubleSupplier mps) {
    m_mps = mps;

    addRequirements(subsystems.notePipeline);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystems.notePipeline.run(m_mps.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystems.notePipeline.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
