package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Controller extends CommandXboxController {

  public Controller(int port) {
    super(port);
  }

  public void setRumble(double strength) {
    this.getHID().setRumble(RumbleType.kBothRumble, strength);
  }

  public RunCommand rumbleCommand(double strength) {
    System.out.println("rumble " + strength);
    return new RunCommand(
      () -> this.getHID().setRumble(RumbleType.kBothRumble, strength)
    );
  }

  public FunctionalCommand intakeRumbleCommand() {
    return new FunctionalCommand(
      // init
      () -> setRumble(1),
      // execute
      () -> {return;},
      // end
      interrupted -> setRumble(0),
      // finished
      () -> false
    );
  }
}
