// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.led.CANdle;

public class LEDSubsystem extends SubsystemBase {

  private final CANdle m_candle = new CANdle(Constants.LEDConstants.kCANdleID);

  private static LEDSubsystem m_instance;

  public static LEDSubsystem getInstance() {
    if (m_instance == null)
      m_instance = new LEDSubsystem();
    
    return m_instance;
  }

  /** Creates a new LedSubsystem. */
  public LEDSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
