// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.notePipeline;

import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NotePipelineSubsystem extends SubsystemBase {
  private final Feeder m_feeder = new Feeder();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();

  /** Creates a new NotePipelineSubsystem. */
  public NotePipelineSubsystem() {}

  private static NotePipelineSubsystem m_instance;
  public static NotePipelineSubsystem getInstance() {
    if (m_instance == null)
      m_instance = new NotePipelineSubsystem();
    return m_instance;
  }

  public void run(double mps) {
    mps = Math.max(-1, Math.min(1, mps));
    
    m_feeder.setMotor(mps);
    m_shooter.setMotor(mps);
    m_intake.setMotor(mps);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
