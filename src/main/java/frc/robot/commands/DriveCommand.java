package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.subsystems;

import java.util.function.DoubleSupplier;


public class DriveCommand extends Command{
  private DoubleSupplier m_xSpeed;
  private DoubleSupplier m_ySpeed;
  private DoubleSupplier m_rot;

  public DriveCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot){
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_rot = rot;

    addRequirements(subsystems.drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    subsystems.drive.drive(
      m_xSpeed.getAsDouble(), 
      m_ySpeed.getAsDouble(), 
      m_rot.getAsDouble()
    );

  }

  @Override
  public void end(boolean interrupted) {
    subsystems.drive.drive(0,0,0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}