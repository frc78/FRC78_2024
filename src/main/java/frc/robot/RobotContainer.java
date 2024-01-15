// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Systems.*;
import frc.robot.Systems.Chassis.*;
import frc.robot.Commands.*;
import frc.robot.Constants.Constants;

public class RobotContainer {
  //private Chassis m_chassis;
  private XboxController m_driveController;
  private Shooter m_shoot;

  public RobotContainer() {
    //m_chassis = new Chassis();

    m_shoot = new Shooter();

    m_driveController = new XboxController(0);

    // m_chassis.setDefaultCommand(new Drive(
    //   m_chassis,
    //   m_driveController::getLeftY,
    //   m_driveController::getLeftX,
    //   m_driveController::getRightX,
    //   m_driveController::getLeftTriggerAxis,
    //   m_driveController::getRightTriggerAxis,
    //   m_driveController::getYButton,
    //   m_driveController::getBButton,
    //   m_driveController::getAButton,
    //   m_driveController::getXButton
    //   ));

    configureBindings();
  }

  private void configureBindings() {
    new Trigger(m_driveController::getAButton).onTrue(new Shoot(m_shoot));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}