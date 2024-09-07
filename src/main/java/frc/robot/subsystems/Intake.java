// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private TalonFX intakeTop;
  private TalonFX intakeBottom;

  private double intakeSpeed;
  private double outtakeSpeed;

  /** Creates a new Intake. */
  public Intake(int intakeTopId, int intakeBottomId, double intakeSpeed, double outtakeSpeed) {
    intakeTop = new TalonFX(intakeTopId, "*");
    intakeBottom = new TalonFX(intakeBottomId, "*");

    TalonFXConfiguration intakeTopConfiguration = new TalonFXConfiguration();
    TalonFXConfiguration intakeBottomConfiguration = new TalonFXConfiguration();

    intakeTopConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeBottomConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    intakeTop.getConfigurator().apply(intakeTopConfiguration);
    intakeBottom.getConfigurator().apply(intakeBottomConfiguration);

    this.intakeSpeed = intakeSpeed;
    this.outtakeSpeed = outtakeSpeed;

    intakeTop.getDutyCycle().setUpdateFrequency(50);
    intakeBottom.getDutyCycle().setUpdateFrequency(50);
    intakeTop.optimizeBusUtilization();
    intakeBottom.optimizeBusUtilization();

    SmartDashboard.putData(this);
  }

  /* intake speed is same for top and bottom */
  public void setIntake(double speed) {
    intakeTop.set(speed);
    intakeBottom.set(speed);
  }

  public Command intakeCommand() {
    return this.startEnd(() -> this.setIntake(intakeSpeed), () -> this.setIntake(0))
        .withName("intake");
  }

  public Command outtakeCommand() {
    return this.startEnd(() -> this.setIntake(outtakeSpeed), () -> this.setIntake(0))
        .withName("outtake");
  }

  public boolean hasNote() {
    return false; // this will change - we need sensor implementation
  }
}
