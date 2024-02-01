// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private CANSparkMax ElevNeoMotor1;
  private CANSparkMax ElevNeoMotor2;

  /** Creates a new Elevator. */
  public Elevator() {
    ElevNeoMotor1 = new CANSparkMax(11, MotorType.kBrushless);
    ElevNeoMotor2 = new CANSparkMax(12, MotorType.kBrushless);

    ElevNeoMotor1.restoreFactoryDefaults();
    ElevNeoMotor2.restoreFactoryDefaults();
    
    ElevNeoMotor1.getAlternateEncoder(8192).setPositionConversionFactor(5.498);

    ElevNeoMotor1.setInverted(true);

    ElevNeoMotor2.follow(ElevNeoMotor1, true);
  }

  public void SetSpeed(double speed){
    ElevNeoMotor1.set(speed);
  }

  public Command setElevatorSpeedUp(double speed){
    return this.runOnce(() -> ElevNeoMotor1.set(speed));
  }

  public Command setElevatorSpeedDown(double speed){
    return this.runOnce(() -> ElevNeoMotor1.set(speed * -1));
  }

  public Command stopElevator(){
    return this.runOnce(() -> ElevNeoMotor1.set(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
