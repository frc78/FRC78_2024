// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private CANSparkMax elevNeoMotor1;
  private CANSparkMax elevNeoMotor2;
  private boolean zeroed = false;

  public boolean hasNotBeenZeroed() {
    return !zeroed;
  }

  private RelativeEncoder encoder;

  /** Creates a new Elevator. */
  private final SparkLimitSwitch magneticLimitSwitch;

  public Elevator() {
    elevNeoMotor1 = new CANSparkMax(11, MotorType.kBrushless);
    elevNeoMotor2 = new CANSparkMax(12, MotorType.kBrushless);

    elevNeoMotor1.restoreFactoryDefaults();
    elevNeoMotor2.restoreFactoryDefaults();

    elevNeoMotor1.setIdleMode(IdleMode.kBrake);
    elevNeoMotor2.setIdleMode(IdleMode.kBrake);

    encoder = elevNeoMotor1.getAlternateEncoder(8192);
    encoder.setPositionConversionFactor(5.498);
    elevNeoMotor1.getPIDController().setFeedbackDevice(encoder);
    elevNeoMotor1.getPIDController().setP(.077);
    elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kForward, false);
    elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kReverse, false);

    elevNeoMotor1.setInverted(true);
    elevNeoMotor2.follow(elevNeoMotor1, true);
    magneticLimitSwitch = elevNeoMotor2.getReverseLimitSwitch(Type.kNormallyOpen);
    magneticLimitSwitch.enableLimitSwitch(false);

    this.setDefaultCommand(setToTarget(0));
  }

  private Command lowerElevatorUntilLimitReached() {
    return run(() -> elevNeoMotor1.set(-.1)).until(magneticLimitSwitch::isPressed);
  }

  private Command configureMotorsAfterZeroing() {
    return Commands.runOnce(
        () -> {
          encoder.setPosition(0);
          elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kForward, true);
          elevNeoMotor1.enableSoftLimit(SoftLimitDirection.kReverse, true);
          elevNeoMotor1.setSoftLimit(SoftLimitDirection.kForward, 14);
          elevNeoMotor1.setSoftLimit(SoftLimitDirection.kReverse, 0);
          zeroed = true;
        });
  }

  public Command zeroElevator() {
    return lowerElevatorUntilLimitReached().andThen(configureMotorsAfterZeroing());
  }

  public Command moveElevatorUp() {
    return this.run(() -> elevNeoMotor1.set(.1));
  }

  public Command moveElevatorDown() {
    return this.run(() -> elevNeoMotor1.set(-.1));
  }

  public void periodic() {
    SmartDashboard.putBoolean("limit pressed", magneticLimitSwitch.isPressed());
    SmartDashboard.putBoolean("zeroed", zeroed);
    SmartDashboard.putNumber("position", encoder.getPosition());
    SmartDashboard.putBoolean(
        "reverse limit reached", elevNeoMotor1.getFault(FaultID.kSoftLimitRev));
    SmartDashboard.putBoolean(
        "forward limit reached", elevNeoMotor1.getFault(FaultID.kSoftLimitFwd));
  }

  /** Moves elevator to target as long as elevator is zeroed */
  public Command setToTarget(double target) {
    return this.run(
        () -> {
          if (zeroed) elevNeoMotor1.getPIDController().setReference(target, ControlType.kPosition);
        });
  }
}
