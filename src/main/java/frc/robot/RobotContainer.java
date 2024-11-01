// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringTopic;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.notePipeline.EjectNoteCommand;
import frc.robot.commands.notePipeline.IntakeCommand;
import frc.robot.commands.notePipeline.ShooterCommand;
import frc.robot.commands.notePipeline.ShooterRevCommand;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.notePipeline.FeederSubsystem;
import frc.robot.subsystems.notePipeline.IntakeSubsystem;
import frc.robot.subsystems.notePipeline.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  public final class subsystems {
    public static final DriveSubsystem drive = DriveSubsystem.getInstance();
    public static final VisionSubsystem vision = VisionSubsystem.getInstance();
    public static final FeederSubsystem feeder = FeederSubsystem.getInstance();
    public  static final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    public  static final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  }

  public final class controllers {
    public static final DriverController driver = DriverController.getInstance();
    public static final OperatorController operator = OperatorController.getInstance();
  }

  private static RobotContainer m_instance;
  public static RobotContainer getInstance() {
    if (m_instance == null)
      m_instance = new RobotContainer();
    return m_instance;
  }

  protected RobotContainer() {
    subsystems.drive.pathPlannerConfig();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    autoChooser.setDefaultOption("DefaultAuto", AutoBuilder.buildAuto("DefaultAuto"));

    configureBindings();
  }

  private void configureBindings() {
    subsystems.drive.setDefaultCommand(
      new DriveCommand(
        controllers.driver::getXVelocity, 
        controllers.driver::getYVelocity, 
        controllers.driver::getRotationalVelocity)
    );

    controllers.operator.getIntakeTrigger().onTrue(new IntakeCommand());

    controllers.operator.getShootingTrigger().whileTrue(new ShooterCommand());

    // controllers.operator.a().whileTrue(new RunCommand(() -> {subsystems.shooter.setRaw(1);}));
    // controllers.operator.a().whileFalse(new RunCommand(() -> {subsystems.shooter.setRaw(0);}));
    controllers.operator.getRevingTrigger().whileTrue(new ShooterRevCommand());

    controllers.operator.getEjectTrigger().whileTrue(new EjectNoteCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public String getAutonomousName() {
    StringTopic activeAutoTopic = NetworkTableInstance.getDefault().getStringTopic("/SmartDashboard/Auto Chooser/active");
    String activeAutoName = activeAutoTopic.getEntry("defaultValue", PubSubOption.sendAll(false)).get();
  
    return activeAutoName;
  }
}
