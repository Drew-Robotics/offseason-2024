// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.notePipeline;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NotePipelineConstants.MotorSpeeds;
import frc.robot.RobotContainer.subsystems;

public class ShooterCommand extends Command {
  private final BooleanSupplier m_shoot;
  
  /** Creates a new ShooterCommand. */
  public ShooterCommand(BooleanSupplier shoot) {
    m_shoot = shoot;

    addRequirements(subsystems.feeder);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shoot.getAsBoolean()) {
      subsystems.feeder.set(MotorSpeeds.shooterFeed); // Feeds the note into the reving shooter
      subsystems.feeder.intook(subsystems.feeder.hasNote()); // Updates if it has intook based on if a note was detected
    } else subsystems.feeder.stop();
  }
}
