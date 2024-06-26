// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.Util;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private CANSparkMax elevNeoMotor1;
  private CANSparkMax elevNeoMotor2;

  private SparkLimitSwitch reverseLimitSwitch;
  private boolean zeroed = false;

  public boolean hasNotBeenZeroed() {
    return !zeroed;
  }

  private RelativeEncoder encoder;

  private static final double kS = 0.070936;
  private static final double kV = 0.79005;
  private static final double kA = 0.086892;
  private static final double kG = 0.088056;
  // Command loop runs at 50Hz, 20ms period
  private static final double kDt = 0.02;
  private double appliedOutput = 0;

  private final Measure<Velocity<Distance>> manualSpeed = InchesPerSecond.of(1);

  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
  private final ProfiledPIDController profiledPid =
      new ProfiledPIDController(
          180,
          0,
          0,
          new TrapezoidProfile.Constraints(
              InchesPerSecond.of(15), InchesPerSecond.per(Second).of(80)),
          kDt);

  public Elevator() {
    elevNeoMotor1 = new CANSparkMax(11, MotorType.kBrushless);
    elevNeoMotor2 = new CANSparkMax(12, MotorType.kBrushless);

    elevNeoMotor1.restoreFactoryDefaults();
    elevNeoMotor2.restoreFactoryDefaults();

    elevNeoMotor1.setIdleMode(IdleMode.kBrake);
    elevNeoMotor2.setIdleMode(IdleMode.kBrake);

    encoder = elevNeoMotor1.getEncoder();
    encoder.setPositionConversionFactor((1.29 * Math.PI) / 25);
    elevNeoMotor1.getPIDController().setFeedbackDevice(encoder);
    elevNeoMotor1.getPIDController().setP(.144);
    elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kForward, false);
    elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kReverse, false);
    profiledPid.setTolerance(Units.inchesToMeters(0.1));

    reverseLimitSwitch = elevNeoMotor1.getReverseLimitSwitch(Type.kNormallyOpen);

    elevNeoMotor1.setInverted(false);
    elevNeoMotor2.follow(elevNeoMotor1, true);

    Util.setRevStatusRates(elevNeoMotor1, 5, 20, 20, 32767, 32767, 32767, 32767, 32767);
    Util.setRevStatusRates(elevNeoMotor2, 500, 32767, 32767, 32767, 32767, 32767, 32767, 32767);

    this.setDefaultCommand(setToTarget(0));
    SmartDashboard.putData(enableCoastMode());
    SmartDashboard.putData(enableBrakeMode());
    SmartDashboard.putData("Elevator Profile", profiledPid);
    SmartDashboard.putData(this);
  }

  public boolean elevatorIsStowed() {
    return zeroed && encoder.getPosition() <= .5;
  }

  public boolean elevIsAtPos() {
    return profiledPid.atGoal();
  }

  private Command lowerElevatorUntilLimitReached() {
    return run(() -> elevNeoMotor1.set(-.1)).until(() -> reverseLimitSwitch.isPressed());
  }

  private Command configureMotorsAfterZeroing() {
    return runOnce(
            () -> {
              encoder.setPosition(0);
              profiledPid.setGoal(0);
              elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kForward, true);
              elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kReverse, true);
              elevNeoMotor1.setSoftLimit(SoftLimitDirection.kForward, 16.4f);
              elevNeoMotor1.setSoftLimit(SoftLimitDirection.kReverse, 0);
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

  /** Manually move elevator up by gradually moving the setpoint. */
  public Command moveElevatorUp() {
    return Commands.runOnce(
            () ->
                profiledPid.setGoal(
                    encoder.getPosition() + manualSpeed.times(kDt).in(InchesPerSecond)))
        .withName("Move Elevator Up");
  }

  /** Manually move elevator down by gradually moving the setpoint. */
  public Command moveElevatorDown() {
    return Commands.runOnce(
            () ->
                profiledPid.setGoal(
                    encoder.getPosition() - manualSpeed.times(kDt).in(InchesPerSecond)))
        .withName("Move Elevator Down");
  }

  public Command enableCoastMode() {
    return Commands.runOnce(
            () -> {
              elevNeoMotor1.setIdleMode(IdleMode.kCoast);
              elevNeoMotor2.setIdleMode(IdleMode.kCoast);
            })
        .andThen(new PrintCommand("Coast Mode Set On Elevator"))
        .ignoringDisable(true)
        .withName("Enable Elevator Coast");
  }

  public Command enableBrakeMode() {
    return Commands.runOnce(
            () -> {
              elevNeoMotor1.setIdleMode(IdleMode.kBrake);
              elevNeoMotor2.setIdleMode(IdleMode.kBrake);
            })
        .andThen(new PrintCommand("Brake Mode Set On Elevator"))
        .ignoringDisable(true)
        .withName("Enable Elevator Brake");
  }

  public void periodic() {
    Logger.recordOutput("Elevator/limit pressed", reverseLimitSwitch.isPressed());
    Logger.recordOutput("Elevator/zeroed", zeroed);
    Logger.recordOutput("Elevator/position", encoder.getPosition());
    Logger.recordOutput(
        "Elevator/reverse limit reached", elevNeoMotor1.getFault(FaultID.kSoftLimitRev));
    Logger.recordOutput(
        "Elevator/forward limit reached", elevNeoMotor1.getFault(FaultID.kSoftLimitFwd));
    Logger.recordOutput("Elevator/PIDoutput", profiledPid.getPositionError());
    Logger.recordOutput("Elevator/Profile Velocity", profiledPid.getSetpoint().velocity);
    Logger.recordOutput("Elevator/AppliedVoltage", appliedOutput);
    Logger.recordOutput("Elevator/Goal", profiledPid.getSetpoint().position);
  }

  public double getElevatorPos() {
    return encoder.getPosition();
  }

  /** Moves elevator to target as long as elevator is zeroed */
  public Command setToTarget(double target) {
    return runOnce(
            () -> {
              profiledPid.setGoal(Units.inchesToMeters(target));
            })
        .andThen(
            run(
                () -> {
                  if (!zeroed) return;
                  appliedOutput =
                      profiledPid.calculate(Units.inchesToMeters(encoder.getPosition()))
                          + feedforward.calculate(
                              MetersPerSecond.of(profiledPid.getSetpoint().velocity)
                                  .in(InchesPerSecond));
                  elevNeoMotor1.setVoltage(appliedOutput);
                }))
        .withName("setTo[" + target + "]");
  }
}
