// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

public class Chassis extends SubsystemBase {
  public SwerveModule[] modules;
  private Pigeon2 pigeon;

  public ChassisSpeeds setChassisSpeed;
  public SwerveModuleState[] setStates;

  public ChassisSpeeds getChassisSpeed;
  public SwerveModuleState[] getStates;
  public SwerveModulePosition[] getPositions;

  public final SwerveDriveKinematics kinematics;

  public Chassis(
      SwerveModule[] modules,
      SwerveDriveKinematics kinematics,
      int pigeonId,
      PhotonCamera ATCamera) {
    // It reads the number of modules from the RobotConstants
    this.modules = modules;
    this.kinematics = kinematics;

    getChassisSpeed = new ChassisSpeeds();
    setChassisSpeed = new ChassisSpeeds();
    getStates = new SwerveModuleState[4];
    getPositions = new SwerveModulePosition[4];

    pigeon = new Pigeon2(pigeonId);
  }

  public void initializeModules() {
    // This is an example of how we will perform operations on all modules
    for (SwerveModule module : modules) {
      module.initialize();
    }
  }

  public double getGyroRot() {
    return pigeon.getYaw().getValueAsDouble();
  }

  public void setGyroRot(double rot) {
    pigeon.setYaw(rot);
  }

  public SwerveModulePosition[] getPositions() {
    for (int i = 0; i < modules.length; i++) {
      getPositions[i] = modules[i].getPosition();
    }
    return getPositions;
  }

  public SwerveModuleState[] getStates() {
    for (int i = 0; i < modules.length; i++) {
      getStates[i] = modules[i].getState();
    }
    return getStates;
  }

  // There is probably a better way to feed this into the AutoBuilder, but this is
  // simpler for now
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getStates());
  }

  public void convertToStates() {
    setStates = kinematics.toSwerveModuleStates(setChassisSpeed);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    setChassisSpeed = speeds;
    convertToStates();
    drive();
  }

  public void drive() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(setStates[i]);
      SmartDashboard.putNumber(i + " Rot", setStates[i].angle.getRotations());
    }

    Logger.recordOutput("ModuleSet", setStates);
  }
}
