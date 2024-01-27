// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeTOP;
  private CANSparkMax intakeBOTTOM;

  /** Creates a new Intake. */
  public Intake(int intakeTopId, int intakeBottomId) {
    intakeTOP = new CANSparkMax(intakeTopId, MotorType.kBrushless);
    intakeBOTTOM = new CANSparkMax(intakeBottomId, MotorType.kBrushless);

    intakeTOP.restoreFactoryDefaults();
    intakeBOTTOM.restoreFactoryDefaults();

    intakeTOP.setClosedLoopRampRate(5);
    intakeBOTTOM.setClosedLoopRampRate(5);
  }

  public void intakeCTRL(double setSpeedT, double setSpeedB) {
    intakeTOP.set(setSpeedT);
    intakeBOTTOM.set(setSpeedB);
  }

  public void intakeSTOP() {
    intakeTOP.set(0);
    intakeBOTTOM.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
