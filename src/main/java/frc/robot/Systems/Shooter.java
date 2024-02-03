// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Shooter extends SubsystemBase {
  
  private CANSparkMax shooterTOP;
  private CANSparkMax shooterBOTTOM;

  private SparkPIDController shooterTopPID;
  private SparkPIDController shooterBottomPID;

  private RelativeEncoder shooterTopENC;
  private RelativeEncoder shooterBottomENC;

  /** Creates a new Shooter. */ 
  public Shooter() {
    shooterTOP = new CANSparkMax(RobotConstants.FLYWHEEL_TOP_ID, MotorType.kBrushless);
    shooterBOTTOM = new CANSparkMax(RobotConstants.FLYWHEEL_BOTTOM_ID, MotorType.kBrushless);

    //shooterTOP.restoreFactoryDefaults();
    //shooterBOTTOM.restoreFactoryDefaults();

    shooterTopPID = shooterTOP.getPIDController();
    shooterBottomPID = shooterBOTTOM.getPIDController();

    shooterTopENC = shooterTOP.getEncoder();
    shooterBottomENC = shooterBOTTOM.getEncoder();

    shooterTopPID.setP(RobotConstants.FLYWHEEL_TOP_P);
    shooterTopPID.setI(RobotConstants.FLYWHEEL_TOP_I);
    shooterTopPID.setD(RobotConstants.FLYWHEEL_TOP_D);
    shooterTopPID.setFF(RobotConstants.FLYWHEEL_TOP_KF);
    shooterTopPID.setOutputRange(RobotConstants.FLYWHEEL_TOP_MIN, RobotConstants.FLYWHEEL_TOP_MAX);

    shooterBottomPID.setP(RobotConstants.FLYWHEEL_BOTTOM_P);
    shooterBottomPID.setI(RobotConstants.FLYWHEEL_BOTTOM_I);
    shooterBottomPID.setD(RobotConstants.FLYWHEEL_BOTTOM_D);
    shooterBottomPID.setFF(RobotConstants.FLYWHEEL_BOTTOM_KF);
    shooterBottomPID.setOutputRange(RobotConstants.FLYWHEEL_BOTTOM_MIN, RobotConstants.FLYWHEEL_BOTTOM_MAX);

    shooterTOP.setClosedLoopRampRate(5);
    shooterBOTTOM.setClosedLoopRampRate(5);
  }

  public void setPIDReferenceTOP(double setPoint){
    shooterTopPID.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  public void setPIDReferenceBOTTOM(double setPoint){
    shooterBottomPID.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  public void setPIDReferenceBOTH(double setPoint){
    setPIDReferenceTOP(setPoint);
    setPIDReferenceBOTTOM(setPoint);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("TOP SetPoint", shooterTopENC.getVelocity());
    SmartDashboard.putNumber("BOTTOM SetPoint", shooterBottomENC.getVelocity());
  }
}
