package frc.robot.controllers;

import frc.robot.Constants.OIConstants;

public class OperatorController extends Controller {
  public static OperatorController m_intance;

  public static OperatorController getInstance(){
    if (m_intance == null){
        m_intance = new OperatorController(OIConstants.OperatorController.kPort);
    }
    return m_intance;
  }

  public OperatorController(int port) {
    super(port);
  }

  public double getNotePipelineMPS() {
    return getRightTriggerAxis();
  }

  public boolean getShooting() {
    return b().getAsBoolean();
  }

  public boolean getReving() {
    return rightBumper().getAsBoolean();
  }
}
