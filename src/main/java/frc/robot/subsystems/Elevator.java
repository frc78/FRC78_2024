// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private CANSparkMax elevNeoMotor1;
  private CANSparkMax elevNeoMotor2;
  private boolean zeroed = false;

  public boolean hasNotBeenZeroed() {
    return !zeroed;
  }

  /** Creates a new Elevator. */
  private final SparkLimitSwitch reverseLimitSwitch;

  private final RelativeEncoder encoder;

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

    elevNeoMotor1.setInverted(true);
    elevNeoMotor2.follow(elevNeoMotor1, true);
    reverseLimitSwitch = elevNeoMotor2.getReverseLimitSwitch(Type.kNormallyOpen);
  }

  private Command configureMotorsForZeroing() {
    return runOnce(
        () -> {
          elevNeoMotor1.follow(elevNeoMotor2, true);
          reverseLimitSwitch.enableLimitSwitch(false);
          elevNeoMotor2.set(0);
        });
  }

  private Command lowerElevatorUntilLimitReached() {
    return startEnd(() -> elevNeoMotor2.set(-.1), () -> elevNeoMotor2.set(0))
        .until(reverseLimitSwitch::isPressed);
  }

  private Command configureMotorsAfterZeroing() {
    return runOnce(
        () -> {
          encoder.setPosition(0);
          elevNeoMotor2.follow(elevNeoMotor1);
          elevNeoMotor1.set(0);
          reverseLimitSwitch.enableLimitSwitch(false);
          elevNeoMotor1.setSoftLimit(SoftLimitDirection.kForward, 14);
          zeroed = true;
        });
  }

  public Command zeroElevator() {
    return configureMotorsForZeroing()
        .andThen(lowerElevatorUntilLimitReached())
        .andThen(configureMotorsAfterZeroing())
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public void SetSpeed(double speed) {
    elevNeoMotor1.set(speed);
  }

  public Command moveElevatorUp() {
    return this.startEnd(() -> elevNeoMotor1.set(.1), () -> elevNeoMotor1.set(0));
  }

  public Command moveElevatorDown() {
    return this.startEnd(() -> elevNeoMotor1.set(-.1), () -> elevNeoMotor1.set(0));
  }
}
