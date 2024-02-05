// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feedback extends SubsystemBase {
  /** Creates a new Feedback. */
  private CANdle bracelet; 
  public Feedback() {
    bracelet = new CANdle(1);
  }

  public void red() {
    bracelet.setLEDs(255, 0, 0); 
  }

  public void multi(Color color) {
    bracelet.setLEDs(((int)(color.red*255)), ((int)(color.green*255)), ((int)(color.blue*255))); 

  }

  public void off() {
    bracelet.setLEDs(0, 0, 0); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
