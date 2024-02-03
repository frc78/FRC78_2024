// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final TalonFX shooterTop;
  private final TalonFX shooterBottom;

  public Shooter(int shooter1ID, int shooter2ID) {
    shooterTop = new TalonFX(shooter1ID);
    shooterBottom = new TalonFX(shooter2ID);
  }

  public Command setShooter(double speed) {
    return new InstantCommand(() -> {
      shooterTop.set(speed);
      shooterBottom.set(speed);
    });
  }
}
