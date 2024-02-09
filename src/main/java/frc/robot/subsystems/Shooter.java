// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint.MinMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.Structs.FFConstants;

public class Shooter extends SubsystemBase {

  private TalonFX shooterTOP;
  private TalonFX shooterBOTTOM;

  private final VelocityVoltage shooterTopVV;
  private final VelocityVoltage shooterBottomVV;

  private final ShooterConfig config;

  /** Creates a new Shooter. */
  public Shooter(ShooterConfig config) {
    this.config = config;
    shooterTOP = new TalonFX(config.flywheelTopID);
    shooterBOTTOM = new TalonFX(config.flywheelBottomID);

    shooterTOP.getConfigurator().apply(new TalonFXConfiguration());
    shooterBOTTOM.getConfigurator().apply(new TalonFXConfiguration());

    var shooterTopConfigs = new Slot0Configs();
    shooterTOP.setInverted(true);
    shooterTopConfigs.kS = config.flywheelTopFF.kS;
    shooterTopConfigs.kV = config.flywheelTopFF.kV;
    shooterTopConfigs.kP = config.flywheelTopPID.kP;
    shooterTopConfigs.kI = config.flywheelTopPID.kI;
    shooterTopConfigs.kD = config.flywheelTopPID.kD;

    shooterTOP.getConfigurator().apply(shooterTopConfigs);

    var shooterBottomConfigs = new Slot0Configs();
    shooterBOTTOM.setInverted(true);
    shooterBottomConfigs.kS = config.flywheelBottomFF.kS;
    shooterBottomConfigs.kV = config.flywheelBottomFF.kV;
    shooterBottomConfigs.kP = config.flywheelBottomPID.kP;
    shooterBottomConfigs.kI = config.flywheelBottomPID.kI;
    shooterBottomConfigs.kD = config.flywheelBottomPID.kD;

    shooterBOTTOM.getConfigurator().apply(shooterBottomConfigs);

    shooterTopVV = new VelocityVoltage(0).withSlot(0);
    shooterBottomVV = new VelocityVoltage(0).withSlot(0);
  }

  public void setPIDReferenceTOP(double setPoint) {
    shooterTOP.setControl(
        shooterTopVV.withVelocity(setPoint / 60).withFeedForward(config.flywheelTopFF.kA));
  }

  public void setPIDReferenceBOTTOM(double setPoint) {
    shooterBOTTOM.setControl(
        shooterBottomVV
            .withVelocity(setPoint / 60)
            .withFeedForward(
                config.flywheelTopFF.kA)); // TODO should maybe use new variable instead of kA
  }

  public void setPIDReferenceBOTH(double setPoint) {
    setPIDReferenceTOP(setPoint);
    setPIDReferenceBOTTOM(setPoint);
  }

  public void shooterSTOP() {
    shooterTOP.set(0);
    shooterBOTTOM.set(0);
  }

  public Command startShooter(int setPoint) {
    return this.runOnce(() -> this.setPIDReferenceBOTH(setPoint));
  }

  public Command stopCommand() {
    return this.runOnce(this::shooterSTOP);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("TOP SetPoint", shooterTOP.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("BOTTOM SetPoint", shooterBOTTOM.getVelocity().getValueAsDouble());
  }

  public static class ShooterConfig {
    public int flywheelTopID;
    public int flywheelBottomID;
    public MinMax flywheelTopMinMax;
    public MinMax flywheelBottomMinMax;
    public PIDConstants flywheelTopPID;
    public PIDConstants flywheelBottomPID;
    public FFConstants flywheelTopFF;
    public FFConstants flywheelBottomFF;

    // Hood's angle of elevation in degrees
    public double HOOD_ANGLE;

    public ShooterConfig(
        int flywheelTopID,
        int flywheelBottomID,
        MinMax flywheelTopMinMax,
        MinMax flywheelBottomMinMax,
        PIDConstants flywheelTopPID,
        PIDConstants flywheelBottomPID,
        FFConstants flywheelTopFF,
        FFConstants flywheelBottomFF) {
      this.flywheelTopID = flywheelTopID;
      this.flywheelBottomID = flywheelBottomID;
      this.flywheelTopMinMax = flywheelTopMinMax;
      this.flywheelBottomMinMax = flywheelBottomMinMax;
      this.flywheelTopPID = flywheelTopPID;
      this.flywheelBottomPID = flywheelBottomPID;
      this.flywheelTopFF = flywheelTopFF;
      this.flywheelBottomFF = flywheelBottomFF;
    }
  }
}
