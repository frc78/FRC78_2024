// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static com.revrobotics.CANSparkBase.*;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  private CANSparkMax wristNeo;
  private AbsoluteEncoder encoder;
  private double stowPos = 50;
  private double target = 0;

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.of(.5).per(Second), Volts.of(2), Seconds.of(10)),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) -> wristNeo.setVoltage(volts.in(Volts)),
              null,
              this,
              "wrist"));

  /** Creates a new Wrist. */
  public Wrist(int WRIST_ID, float WRIST_HIGH_LIM, float WRIST_LOW_LIM) {
    wristNeo = new CANSparkMax(WRIST_ID, MotorType.kBrushless);

    wristNeo.restoreFactoryDefaults();

    wristNeo.setIdleMode(IdleMode.kBrake);

    encoder = wristNeo.getAbsoluteEncoder(Type.kDutyCycle);
    encoder.setPositionConversionFactor(360);
    encoder.setVelocityConversionFactor(360.0 / 60.0);
    wristNeo.getPIDController().setFeedbackDevice(encoder);
    wristNeo.getPIDController().setP(.03);

    encoder.setInverted(true);
    encoder.setZeroOffset(0);

    wristNeo.setSoftLimit(SoftLimitDirection.kForward, WRIST_HIGH_LIM);
    wristNeo.setSoftLimit(SoftLimitDirection.kReverse, WRIST_LOW_LIM);

    wristNeo.enableSoftLimit(SoftLimitDirection.kForward, true);
    wristNeo.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  public Command setToTarget(double target) {
    this.target = target;
    return runOnce(() -> wristNeo.getPIDController().setReference(target, ControlType.kPosition));
  }

  public Command stow() {
    return setToTarget(stowPos);
  }

  public boolean isAtTarget() {
    return Math.abs(target - encoder.getPosition()) < 2;
  }

  private void configureMotorsBeforeSysId() {
    // Applied Output - Combined with motor voltage gives us applied voltage
    wristNeo.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10);
    // Motor Voltage
    wristNeo.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 10);
    // Absolute Encoder Position
    wristNeo.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 10);
    // Absolute Encoder Velocity
    wristNeo.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 10);
  }

  private void configureMotorsAfterSysId() {
    // Applied Output
    wristNeo.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 65535);
    // Motor Voltage
    wristNeo.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 65535);
    // Absolute Encoder Position - Keep at 200 to update telemetry
    wristNeo.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 200);
    // Absolute Encoder Velocity
    wristNeo.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 65535);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return runOnce(this::configureMotorsBeforeSysId)
        .andThen(sysIdRoutine.quasistatic(direction))
        .finallyDo(this::configureMotorsAfterSysId);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return runOnce(this::configureMotorsBeforeSysId)
        .andThen(sysIdRoutine.dynamic(direction))
        .finallyDo(this::configureMotorsAfterSysId);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Wrist Enc Pos", encoder.getPosition());
  }
}
