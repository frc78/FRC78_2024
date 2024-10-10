// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  private TalonFX motor;
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  private Canandmag encoder;
  private Measure<Angle> stowPos = Degrees.of(55);

  private boolean motorSyncedWithEncoder = false;

  /** Creates a new Wrist. */
  public Wrist(int WRIST_ID, Measure<Angle> WRIST_HIGH_LIM, Measure<Angle> WRIST_LOW_LIM) {
    motor = new TalonFX(WRIST_ID, "*");

    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

    motorConfiguration.Feedback.SensorToMechanismRatio = 375;

    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    encoder = new Canandmag(1);
    Canandmag.Settings settings = new Canandmag.Settings();
    settings.setInvertDirection(true);
    encoder.getInternalSettingsManager().setSettings(settings, -1);

    motorConfiguration.ClosedLoopGeneral.ContinuousWrap = true;
    motorConfiguration.Slot0.kP = 500;

    motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = WRIST_LOW_LIM.in(Rotations);
    motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = WRIST_HIGH_LIM.in(Rotations);
    motor.getConfigurator().apply(motorConfiguration);

    motor.getPosition().setUpdateFrequency(10);
    motor.getMotorVoltage().setUpdateFrequency(10);
    motor.getVelocity().setUpdateFrequency(10);
    motor.getClosedLoopError().setUpdateFrequency(10);

    SmartDashboard.putData(this);
    SmartDashboard.putData(enableBrakeMode());
    SmartDashboard.putData(enableCoastMode());

    // Set the talon internal encoder to absolute encoder position

    if (encoder.isConnected()) {
      motor.setPosition(encoder.getAbsPosition());
      motorSyncedWithEncoder = true;
    } else {
      motorSyncedWithEncoder = false;
    }

    setDefaultCommand(stow().andThen(Commands.idle()));
  }

  public Command setToTargetCmd(Measure<Angle> target) {
    return runOnce(() -> setToTarget(target))
      .withName("setGoal[" + target + "]")
      .andThen(Commands.idle());
  }

  public void setToTarget(Measure<Angle> target) {
    if (motorSyncedWithEncoder) {
      motor.setControl(m_positionVoltage.withPosition(target.in(Rotations)));
    } else {
      motor.stopMotor();
    }
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

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              Seconds.of(5),
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> motor.setVoltage(volts.magnitude()), null, this, "wrist"));

  public Command sysId() {
    return Commands.sequence(
        sysIdRoutine
            .quasistatic(SysIdRoutine.Direction.kReverse)
            .until(() -> motor.getFault_ReverseSoftLimit().getValue()),
        sysIdRoutine
            .quasistatic(SysIdRoutine.Direction.kForward)
            .until(() -> motor.getFault_ForwardSoftLimit().getValue()),
        sysIdRoutine
            .dynamic(SysIdRoutine.Direction.kReverse)
            .until(() -> motor.getFault_ReverseSoftLimit().getValue()),
        sysIdRoutine
            .dynamic(SysIdRoutine.Direction.kForward)
            .until(() -> motor.getFault_ReverseSoftLimit().getValue()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!motorSyncedWithEncoder) {
      if (encoder.isConnected()) {
        motor.setPosition(encoder.getAbsPosition());
        motorSyncedWithEncoder = true;
      }
    }

    Logger.recordOutput("Wrist Enc Pos", encoder.getPosition());
    Logger.recordOutput("Wrist Abs Enc Pos", encoder.getAbsPosition());
    Logger.recordOutput("Wrist Motor Position", motor.getPosition().getValue());
  }
}
