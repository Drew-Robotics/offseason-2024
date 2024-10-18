// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.notePipeline;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NotePipelineConstants.MotorSpeeds;
import frc.robot.RobotContainer.subsystems;

public class ShooterRevCommand extends Command {
  private final BooleanSupplier m_reving;

  /** Creates a new ShooterRevCommand. */
  public ShooterRevCommand(BooleanSupplier reving) {
    m_reving = reving;
    
    addRequirements(subsystems.shooter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_reving.getAsBoolean()) subsystems.shooter.set(MotorSpeeds.shooterRev);
    else subsystems.shooter.stop(); // revs the shooter
  }
}