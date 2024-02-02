// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeTop;
  private CANSparkMax intakeBottom;

  private double intakeSpeed;
  private double outtakeSpeed;

  /** Creates a new Intake. */
  public Intake(int intakeTopId, int intakeBottomId, double intakeSpeed, double outtakeSpeed) {
    intakeTop = new CANSparkMax(intakeTopId, MotorType.kBrushless);
    intakeBottom = new CANSparkMax(intakeBottomId, MotorType.kBrushless);

    intakeTop.restoreFactoryDefaults();
    intakeBottom.restoreFactoryDefaults();

    this.intakeSpeed = intakeSpeed;
    this.outtakeSpeed = outtakeSpeed;

    // intakeTop.setClosedLoopRampRate(5);
    // intakeBottom.setClosedLoopRampRate(5);
  }

  public void intakeControl(double setSpeedT, double setSpeedB) {
    intakeTop.set(setSpeedT);
    intakeBottom.set(setSpeedB);
  }

  // intake speed is same for top and bottom
  public Command intakeCommand() {
    return this.startEnd(
        () -> this.intakeControl(intakeSpeed, intakeSpeed), () -> this.intakeStop());
  }

  public Command outtakeCommand() {
    return this.startEnd(
        () -> this.intakeControl(outtakeSpeed, outtakeSpeed), () -> this.intakeStop());
  }

  public void intakeStop() {
    intakeTop.set(0);
    intakeBottom.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
