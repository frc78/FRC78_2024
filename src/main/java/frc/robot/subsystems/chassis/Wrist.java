// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {

  private CANSparkMax wristNeo;
  private AbsoluteEncoder encoder;

  private double highLim;
  private double lowLim;

  /** Creates a new Wrist. */
  public Wrist(int WRIST_ID, float WRIST_HIGH_LIM, float WRIST_LOW_LIM) {
    wristNeo = new CANSparkMax(WRIST_ID, MotorType.kBrushless);

    wristNeo.restoreFactoryDefaults();

    wristNeo.setIdleMode(IdleMode.kBrake);

    encoder = wristNeo.getAbsoluteEncoder(Type.kDutyCycle);
    encoder.setPositionConversionFactor(360);
    wristNeo.getPIDController().setFeedbackDevice(encoder);

    encoder.setInverted(true);
    encoder.setZeroOffset(340);

    wristNeo.setSoftLimit(SoftLimitDirection.kForward, WRIST_HIGH_LIM);
    wristNeo.setSoftLimit(SoftLimitDirection.kReverse, WRIST_LOW_LIM);

    wristNeo.enableSoftLimit(SoftLimitDirection.kForward, true);
    wristNeo.enableSoftLimit(SoftLimitDirection.kReverse, true);

    highLim = WRIST_HIGH_LIM;
    lowLim = WRIST_LOW_LIM;
  }

  public void SetSpeed(double speed) {
    wristNeo.set(speed);
  }

  public boolean atHighLimit() {
    double wristPosition = encoder.getPosition();
    if (wristPosition >= highLim) {
      return true;
    } else {
      return false;
    }
  }

  public boolean atLowLimit() {
    double wristPosition = encoder.getPosition();
    if (wristPosition <= lowLim) {
      return true;
    } else {
      return false;
    }
  }

  public Command moveWristUp() {
    return this.startEnd(() -> wristNeo.set(.3), () -> wristNeo.set(0));
  }

  public Command moveWristDown() {
    return this.startEnd(() -> wristNeo.set(-.3), () -> wristNeo.set(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
