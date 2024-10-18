// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.notePipeline;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NotePipelineConstants.MotorSpeeds;
import frc.robot.RobotContainer.subsystems;

public class IntakeCommand extends Command {
  private final BooleanSupplier m_run;
  
  /** Creates a new IntakeCommand. */
  public IntakeCommand(BooleanSupplier run) {
    m_run = run;

    addRequirements(subsystems.intake, subsystems.feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystems.feeder.set(MotorSpeeds.feeder);
    subsystems.intake.set(MotorSpeeds.intake);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystems.intake.stop();
    subsystems.shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (subsystems.feeder.hasNote()) {  // stops when the sensor detects a note or the cancel button is pressed, and sets intook to true
      subsystems.feeder.intook(true);
      return true;
    } else {
      return false;
    }
  }
}
