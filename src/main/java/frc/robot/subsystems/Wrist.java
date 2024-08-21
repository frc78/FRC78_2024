// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  private TalonFX motor;
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  private Canandmag encoder;
  private double stowPos = 55;
  private double target = 0;

  /** Creates a new Wrist. */
  public Wrist(int WRIST_ID, float WRIST_HIGH_LIM, float WRIST_LOW_LIM) {
    motor = new TalonFX(WRIST_ID, "*");

    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

    motorConfiguration.Feedback.SensorToMechanismRatio = 108.0 / 360;

    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    encoder = new Canandmag(1);
    Canandmag.Settings settings = new Canandmag.Settings();
    settings.setInvertDirection(true);
    encoder.getInternalSettingsManager().setSettings(settings, -1);
    motorConfiguration.ClosedLoopGeneral.ContinuousWrap = true;
    motorConfiguration.Slot0.kP = .03;

    settings.setInvertDirection(true);

    motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = WRIST_LOW_LIM;
    motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = WRIST_HIGH_LIM;

    motor.getPosition().setUpdateFrequency(50);
    motor.getRotorPosition().setUpdateFrequency(50);
    motor.optimizeBusUtilization();

    motor.getConfigurator().apply(motorConfiguration);

    SmartDashboard.putData(this);
    SmartDashboard.putData(enableBrakeMode());
    SmartDashboard.putData(enableCoastMode());

    // Read the current position of the absolute encoder (redux encoder)
    double abspos = encoder.getAbsPosition();

    // Set the talon internal encoder to absolute encoder position
    motor.setPosition(abspos);
  }

  public Command setToTargetCmd(double target) {
    this.target = target;
    return runOnce(() -> motor.setControl(m_positionVoltage.withPosition(target)))
        .withName("setGoal[" + target + "]");
  }

  public void setToTarget(double target) {
    this.target = target;
    motor.setControl(m_positionVoltage.withPosition(target));
  }

  public Command stow() {
    return setToTargetCmd(stowPos).withName("Stow").withName("Stow");
  }

  public Command enableCoastMode() {
    MotorOutputConfigs config = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast);
    return Commands.runOnce(() -> motor.getConfigurator().apply(config))
        .andThen(new PrintCommand("Coast Mode Set On Wrist"))
        .ignoringDisable(true)
        .withName("Enable Wrist Coast");
  }

  public Command enableBrakeMode() {
    MotorOutputConfigs config = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    return Commands.runOnce(() -> motor.getConfigurator().apply(config))
        .andThen(new PrintCommand("Brake Mode Set On Wrist"))
        .ignoringDisable(true)
        .withName("Enable Wrist Brake");
  }

  public boolean isAtTarget() {
    return Math.abs(target - encoder.getPosition()) < 2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Wrist Enc Pos", encoder.getPosition());
    Logger.recordOutput("Wrist Abs Enc Pos", encoder.getAbsPosition());
    Logger.recordOutput("Wrist Motor Position", motor.getPosition().getValue());
  }
}
