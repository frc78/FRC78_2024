// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private TalonFX leader;
  private TalonFX follower;

  MotionMagicVoltage control = new MotionMagicVoltage(0);

  // private SparkLimitSwitch reverseLimitSwitch;
  private boolean zeroed = false;

  public boolean hasNotBeenZeroed() {
    return !zeroed;
  }

  public Elevator() {
    leader = new TalonFX(11, "*");
    follower = new TalonFX(12, "*");

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Feedback.SensorToMechanismRatio = 25 / (1.29 * Math.PI);
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    configs.Slot0.kP = 66.84;
    configs.Slot0.kI = 0;
    configs.Slot0.kD = 1.7421;
    configs.Slot0.kS = 0.22964;
    configs.Slot0.kV = 0.70964;
    configs.Slot0.kA = 0.018805;
    configs.Slot0.kG = 0.12011;
    configs.MotionMagic.MotionMagicAcceleration = 80f;
    configs.MotionMagic.MotionMagicCruiseVelocity = 15f;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    leader.getConfigurator().apply(configs);
    follower.setControl(new Follower(11, true));

    leader.setNeutralMode(NeutralModeValue.Brake);
    follower.setNeutralMode(NeutralModeValue.Brake);

    // reverseLimitSwitch = leader.getReverseLimitSwitch(Type.kNormallyOpen);

    // leader.setInverted(false);
    // follower.Follow(11, false);
    // Util.setRevStatusRates(leader, 5, 20, 20, 32767, 32767, 32767, 32767, 32767);
    // Util.setRevStatusRates(follower, 500, 32767, 32767, 32767, 32767, 32767, 32767, 32767);

    leader.getPosition().setUpdateFrequency(50);

    this.setDefaultCommand(setToTarget(0));
    SmartDashboard.putData(enableCoastMode());
    SmartDashboard.putData(enableBrakeMode());
    SmartDashboard.putData(this);
  }

  public boolean elevatorIsStowed() {
    return zeroed && leader.getPosition().getValue() <= .5;
  }

  public boolean elevIsAtPos() {
    var actualPos = leader.getPosition().getValue();
    var targetPos = control.Position;

    return Math.abs(actualPos - targetPos) <= 0.1f;
  }

  private Command lowerElevatorUntilLimitReached() {
    return run(() -> leader.set(-.1))
        .until(() -> leader.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround);
  }

  private Command configureMotorsAfterZeroing() {
    return runOnce(
            () -> {
              var limitConfigs =
                  new SoftwareLimitSwitchConfigs()
                      .withForwardSoftLimitEnable(true)
                      .withForwardSoftLimitThreshold(16f)
                      .withReverseSoftLimitEnable(true)
                      .withReverseSoftLimitThreshold(0);

              leader.getConfigurator().apply(limitConfigs);

              leader.setPosition(0);
              zeroed = true;
              this.setDefaultCommand(setToTarget(0));
            })
        .withName("Configure Motors After Zeroing");
  }

  public Command zeroElevator() {
    return lowerElevatorUntilLimitReached()
        .andThen(configureMotorsAfterZeroing())
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
        .withName("Zero Elevator");
  }

  public Command enableCoastMode() {
    return Commands.runOnce(
            () -> {
              leader.setNeutralMode(NeutralModeValue.Coast);
              follower.setNeutralMode(NeutralModeValue.Coast);
            })
        .andThen(new PrintCommand("Coast Mode Set On Elevator"))
        .ignoringDisable(true)
        .withName("Enable Elevator Coast");
  }

  public Command enableBrakeMode() {
    return Commands.runOnce(
            () -> {
              leader.setNeutralMode(NeutralModeValue.Brake);
              follower.setNeutralMode(NeutralModeValue.Brake);
            })
        .andThen(new PrintCommand("Brake Mode Set On Elevator"))
        .ignoringDisable(true)
        .withName("Enable Elevator Brake");
  }

  public void periodic() {
    Logger.recordOutput(
        "Elevator/limit pressed",
        leader.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround);
    Logger.recordOutput("Elevator/zeroed", zeroed);
    Logger.recordOutput("Elevator/position", leader.getPosition().getValue());
  }

  public double getElevatorPos() {
    return leader.getPosition().getValue();
  }

  /** Moves elevator to target as long as elevator is zeroed */
  public Command setToTarget(double target) {
    return runOnce(
            () -> {
              if (zeroed) {
                control.Position = target;
                leader.setControl(control);
              }
            })
        .andThen(Commands.idle())
        .withName("setTo[" + target + "]");
  }

  private final TalonFX m_motor = new TalonFX(0);
  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Units.Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> leader.setControl(m_voltReq.withOutput(volts.in(Units.Volts))),
              null,
              this));

  public Command runSysId() {
    return Commands.sequence(
        runOnce(SignalLogger::start),
        m_sysIdRoutine
            .quasistatic(Direction.kForward)
            .until(() -> leader.getFault_ForwardSoftLimit().getValue()),
        m_sysIdRoutine
            .quasistatic(Direction.kReverse)
            .until(() -> leader.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround),
        m_sysIdRoutine
            .dynamic(Direction.kForward)
            .until(() -> leader.getFault_ForwardSoftLimit().getValue()),
        m_sysIdRoutine
            .dynamic(Direction.kReverse)
            .until(() -> leader.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround),
        runOnce(SignalLogger::stop));
  }
}
