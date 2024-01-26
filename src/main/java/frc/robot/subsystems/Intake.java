// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeTOP;
  private CANSparkMax intakeBOTTOM;

  private SparkPIDController intakeTopPID;
  private SparkPIDController intakeBottomPID;

  private RelativeEncoder intakeTopENC;
  private RelativeEncoder intakeBottomENC;

  /** Creates a new Intake. */
  public Intake(
      int topId, int bottomId, double topKp, double topKf, double bottomKp, double bottomKf) {
    intakeTOP = new CANSparkMax(topId, CANSparkLowLevel.MotorType.kBrushless);
    intakeBOTTOM = new CANSparkMax(bottomId, CANSparkLowLevel.MotorType.kBrushless);

    intakeTOP.restoreFactoryDefaults();
    intakeBOTTOM.restoreFactoryDefaults();

    intakeTopPID = intakeTOP.getPIDController();
    intakeBottomPID = intakeBOTTOM.getPIDController();

    intakeTopENC = intakeTOP.getEncoder();
    intakeBottomENC = intakeBOTTOM.getEncoder();

    intakeTopPID.setP(topKp);
    intakeTopPID.setFF(topKf);

    intakeBottomPID.setP(bottomKp);
    intakeBottomPID.setFF(bottomKf);

    // .5 seconds from 0-full speed
    intakeTOP.setClosedLoopRampRate(.5);
    intakeBOTTOM.setClosedLoopRampRate(.5);
  }

  public void setPIDReferenceTOP(double setPoint) {
    intakeTopPID.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  public void setPIDReferenceBOTTOM(double setPoint) {
    intakeBottomPID.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  public void setPIDReferenceBOTH(double setPoint) {
    setPIDReferenceTOP(setPoint);
    setPIDReferenceBOTTOM(setPoint);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("TOP SetPoint", intakeTopENC.getVelocity());
    SmartDashboard.putNumber("BOTTOM SetPoint", intakeBottomENC.getVelocity());
  }
}
