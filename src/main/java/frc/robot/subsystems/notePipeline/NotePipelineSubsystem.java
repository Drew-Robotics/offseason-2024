// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.notePipeline;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Subsystem;

public class NotePipelineSubsystem extends Subsystem {
  private final Feeder m_feeder = new Feeder();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();

  private final DoubleSupplier m_feederVel = m_feeder.getVelocity();
  private final DoubleSupplier m_intakeVel = m_intake.getVelocity();
  private final DoubleSupplier[] m_shooterVels = m_shooter.getVelocity();

  /** Creates a new NotePipelineSubsystem. */
  public NotePipelineSubsystem() {
    super("Note Pipeline Subsystem");
  }

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
  public void periodic() {}

  public void dashboardInit() {}
  public void publishInit() {}
  public void publishPeriodic() {}

  public void dashboardPeriodic() {
    SmartDashboard.putNumber("Left Shooter Vel", m_shooterVels[0].getAsDouble());
    SmartDashboard.putNumber("Right Shooter Vel", m_shooterVels[1].getAsDouble());
    SmartDashboard.putNumber("Intake Vel", m_intakeVel.getAsDouble());
    SmartDashboard.putNumber("Feeder Vel", m_feederVel.getAsDouble());
  }
}
