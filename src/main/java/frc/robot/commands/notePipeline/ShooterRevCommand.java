// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.notePipeline;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NotePipelineConstants.MotorSpeeds;
import frc.robot.RobotContainer.subsystems;

public class ShooterRevCommand extends Command {

  /** Creates a new ShooterRevCommand. */
  public ShooterRevCommand() {
    addRequirements(subsystems.shooter);
  }

  @Override
  public void initialize() {
    subsystems.shooter.set(MotorSpeeds.shooterRev);
  }

  @Override
  public void end(boolean interrupted) {
    subsystems.shooter.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(m_reving.getAsBoolean());
    // if (m_reving.getAsBoolean()) subsystems.shooter.setRaw(MotorSpeeds.shooterRev);
    // else subsystems.shooter.stop(); // revs the shooter
  }
}
