// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;

public class Shooter extends SubsystemBase {
  
  private TalonFX shooterTOP;
  private TalonFX shooterBOTTOM;

  private final VelocityVoltage shooterTopVV;
  private final VelocityVoltage shooterBottomVV;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterTOP = new TalonFX(RobotConstants.FLYWHEEL_TOP_ID);
    shooterBOTTOM = new TalonFX(RobotConstants.FLYWHEEL_BOTTOM_ID);

    shooterTOP.getConfigurator().apply(new TalonFXConfiguration());
    shooterBOTTOM.getConfigurator().apply(new TalonFXConfiguration());

    var shooterTopConfigs = new Slot0Configs();
    shooterTopConfigs.kS = RobotConstants.FLYWHEEL_TOP_S;
    shooterTopConfigs.kV = RobotConstants.FLYWHEEL_TOP_V;
    shooterTopConfigs.kP = RobotConstants.FLYWHEEL_TOP_P;
    shooterTopConfigs.kI = RobotConstants.FLYWHEEL_TOP_I;
    shooterTopConfigs.kD = RobotConstants.FLYWHEEL_TOP_D;

    shooterTOP.getConfigurator().apply(shooterTopConfigs);

    var shooterBottomConfigs = new Slot0Configs();
    shooterBottomConfigs.kS = RobotConstants.FLYWHEEL_BOTTOM_S;
    shooterBottomConfigs.kV = RobotConstants.FLYWHEEL_BOTTOM_V;
    shooterBottomConfigs.kP = RobotConstants.FLYWHEEL_BOTTOM_P;
    shooterBottomConfigs.kI = RobotConstants.FLYWHEEL_BOTTOM_I;
    shooterBottomConfigs.kD = RobotConstants.FLYWHEEL_BOTTOM_D;

    shooterBOTTOM.getConfigurator().apply(shooterBottomConfigs);

    shooterTopVV = new VelocityVoltage(0).withSlot(0);
    shooterBottomVV = new VelocityVoltage(0).withSlot(0);
  }

  public void setPIDReferenceTOP(double setPoint){
    shooterTOP.setControl(shooterTopVV.withVelocity(setPoint * 60).withFeedForward(RobotConstants.FLYWHEEL_TOP_FF));
  }

  public void setPIDReferenceBOTTOM(double setPoint){
    shooterBOTTOM.setControl(shooterBottomVV.withVelocity(setPoint * 60).withFeedForward(RobotConstants.FLYWHEEL_BOTTOM_FF));
  }

  public void setPIDReferenceBOTH(double setPoint){
    setPIDReferenceTOP(setPoint);
    setPIDReferenceBOTTOM(setPoint);
  }

  public void shooterSTOP(){
    shooterTOP.set(0);
    shooterBOTTOM.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("TOP SetPoint", shooterTOP.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("BOTTOM SetPoint", shooterBOTTOM.getVelocity().getValueAsDouble());
  }
}
