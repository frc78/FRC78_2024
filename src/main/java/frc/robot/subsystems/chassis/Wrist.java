// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {

  private CANSparkMax wristNeo;

  /** Creates a new Wrist. */
  public Wrist(int WRIST_ID) {
    wristNeo = new CANSparkMax(WRIST_ID, MotorType.kBrushless);

    wristNeo.restoreFactoryDefaults();

    wristNeo.setIdleMode(IdleMode.kBrake);

    AbsoluteEncoder encoder = wristNeo.getAbsoluteEncoder(Type.kDutyCycle);
    encoder.setPositionConversionFactor(1);
    wristNeo.getPIDController().setFeedbackDevice(encoder);
  }

  public void SetSpeed(double speed) {
    wristNeo.set(speed);
  }

  public Command moveWristUp() {
    return this.startEnd(() -> wristNeo.set(.1), () -> wristNeo.set(0));
  }

  public Command moveWristDown() {
    return this.startEnd(() -> wristNeo.set(-.1), () -> wristNeo.set(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
