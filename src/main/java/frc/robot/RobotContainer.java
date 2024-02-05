// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Systems.Feedback;
import frc.robot.Systems.Chassis.*;
import frc.robot.Commands.*;

public class RobotContainer {
  private Chassis m_chassis;
  private XboxController m_driveController;
  private final SendableChooser<Command> autoChooser;
  private Feedback m_feedback; 

  public RobotContainer() {
    m_chassis = new Chassis();
    m_feedback = new Feedback(); 

    m_driveController = new XboxController(0);

    m_chassis.setDefaultCommand(new Drive(
      m_chassis,
      m_driveController::getLeftY,
      m_driveController::getLeftX,
      m_driveController::getRightX,
      m_driveController::getLeftTriggerAxis,
      m_driveController::getRightTriggerAxis,
      m_driveController::getYButton,
      m_driveController::getBButton,
      m_driveController::getAButton,
      m_driveController::getXButton
      ));

      autoChooser = AutoBuilder.buildAutoChooser();

      SmartDashboard.putData("AutoMode", autoChooser);

      configureBindings();
  }

  private void configureBindings() {
    new Trigger(m_driveController::getStartButton).onTrue(new InstantCommand(() -> m_chassis.resetPose(new Pose2d())));
    new Trigger(m_driveController::getBButton).whileTrue(Commands.startEnd(m_feedback::red, m_feedback::off, m_feedback)); 
    new Trigger(m_driveController::getXButton).whileTrue(Commands.startEnd(()->m_feedback.multi(new Color(186,7,162)), m_feedback::off, m_feedback)); 
    new Trigger(m_driveController::getYButton).whileTrue(Commands.startEnd(()->m_feedback.multi(Color.fromHSV(316,47,10)), m_feedback::off, m_feedback)); 
    new Trigger(m_driveController::getAButton).whileTrue(Commands.startEnd(()->m_feedback.multi(new Color(0,255,0)), m_feedback::off, m_feedback)); 

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}