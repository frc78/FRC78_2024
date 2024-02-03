// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feed extends SubsystemBase {
  private final TalonFX feedMotor;
  public TimeOfFlight feedSensor;

  /** Creates a new Feed. */
  public Feed() {
    feedMotor = new TalonFX(14);
    feedSensor = new TimeOfFlight(17);
    feedSensor.setRangeOfInterest(-1, 1, 1, -1);
  }

  public void setSpeed(double speed) {
    feedMotor.set(speed);
  }

  public Command runFeed() {
    return this.startEnd(() -> feedMotor.set(0.2), () -> feedMotor.set(0));
  }

  public Command reverseFeed() {
    return this.startEnd(() -> feedMotor.set(-.2), () -> feedMotor.set(0));
  }

  public boolean isTriggered() {
    if (feedSensor.getRange() <= 40) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("feedSensor", feedSensor.getRange());
  }
}
