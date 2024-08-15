// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private static final double kS = 0.070936;
  private static final double kV = 0.79005;
  private static final double kA = 0.086892;
  private static final double kG = 0.088056;
  // Command loop runs at 50Hz, 20ms period
  private static final double kDt = 0.02;
  private double appliedOutput = 0;

  public Elevator() {
    leader = new TalonFX(11, "*");
    follower = new TalonFX(12, "*");

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Feedback.SensorToMechanismRatio = (1.29 * Math.PI) / 25;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    configs.Slot0.kP = 1;
    configs.Slot0.kI = 0;
    configs.Slot0.kD = 10;
    configs.Slot0.kV = 2;
    configs.Slot0.kS = 0.070936;
    configs.Slot0.kV = 0.79005;
    configs.Slot0.kA = 0.086892;
    configs.Slot0.kG = 0.088056;
    configs.MotionMagic.MotionMagicAcceleration = 80f;
    configs.MotionMagic.MotionMagicCruiseVelocity = 15f;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leader.getConfigurator().apply(configs);
    follower.setControl(new Follower(11, true));

    leader.setNeutralMode(NeutralModeValue.Brake);
    follower.setNeutralMode(NeutralModeValue.Brake);

    // reverseLimitSwitch = leader.getReverseLimitSwitch(Type.kNormallyOpen);

    // leader.setInverted(false);
    // follower.Follow(11, false);

    follower.setControl(new Follower(11, false));

    // Util.setRevStatusRates(leader, 5, 20, 20, 32767, 32767, 32767, 32767, 32767);
    // Util.setRevStatusRates(follower, 500, 32767, 32767, 32767, 32767, 32767, 32767, 32767);

    leader.getPosition().setUpdateFrequency(50);
    leader.optimizeBusUtilization();

    follower.optimizeBusUtilization();

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
    return run(() -> leader.set(-.1)).until(() -> leader.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround);
  }

  private Command configureMotorsAfterZeroing() {
    return runOnce(
            () -> {
              var limitConfigs = new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true).withForwardSoftLimitThreshold(17f).withReverseSoftLimitEnable(true).withReverseSoftLimitThreshold(0);

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
    Logger.recordOutput("Elevator/limit pressed", leader.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround);
    Logger.recordOutput("Elevator/zeroed", zeroed);
    Logger.recordOutput("Elevator/position", leader.getPosition().getValue());
  }

  public double getElevatorPos() {
    return leader.getPosition().getValue();
  }

  /** Moves elevator to target as long as elevator is zeroed */
  public Command setToTarget(double target) {
    return runOnce(() -> {
      if (zeroed) {
        control.Position = target;
        leader.setControl(control);
      }
    }).withName("setTo[" + target + "]");
  }
}
