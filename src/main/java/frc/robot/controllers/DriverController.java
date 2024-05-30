package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.OIConstants;

public class DriverController extends Controller{

  public static DriverController m_intance;

  public static DriverController getInstance(){
    if (m_intance == null){
        m_intance = new DriverController(OIConstants.DriverController.kPort);
    }
    return m_intance;
  }

  private DriverController(int port){
    super(port);
  }

  public double getXVelocity(){
    return -MathUtil.applyDeadband(this.getLeftY(), OIConstants.DriverController.kDriveDeadband);
  }

  public double getYVelocity(){
    return -MathUtil.applyDeadband(this.getLeftX(), OIConstants.DriverController.kDriveDeadband);
  }

  public double getRotationalVelocity(){
    return -MathUtil.applyDeadband(this.getRightX(), OIConstants.DriverController.kDriveDeadband);
  }

  public Trigger getTurnToZeroButton(){
    return y();
  }

  public Trigger getStopButton(){
    return rightBumper();
  }
}