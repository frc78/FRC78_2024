// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Classes.Util;
import frc.robot.Constants.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Systems.Chassis.Chassis;

public class OrbitalTarget extends Command {
  
  private final Chassis chassis;

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier lTriggerSupplier;
  private final DoubleSupplier rTriggerSupplier;

  private final Translation2d targetPose;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotController;

  // Target pose in field space for the robot to move to
  private double xTarget;
  private double yTarget;
  private double rotTarget;

  // Essentially the polar coordinates of the robot relative to the target
  private double TRAngle; // Target-Robot angle
  private double orbitDistance;

  public OrbitalTarget(Chassis chassis,
    DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier,
    DoubleSupplier lTriggerSupplier, DoubleSupplier rTriggerSupplier) {

    this.chassis = chassis;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.lTriggerSupplier = lTriggerSupplier;
    this.rTriggerSupplier = rTriggerSupplier;

    // Might be shorter way of doing this
    if(DriverStation.getAlliance().isPresent()) {
      targetPose = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Constants.BLUE_ORBIT_POSE : Constants.RED_ORBIT_POSE;
    } else {
      targetPose = Constants.BLUE_ORBIT_POSE;
    }

    xController = new PIDController(RobotConstants.TRANSLATION_PID.kP, RobotConstants.TRANSLATION_PID.kI, RobotConstants.TRANSLATION_PID.kD);
    yController = new PIDController(RobotConstants.TRANSLATION_PID.kP, RobotConstants.TRANSLATION_PID.kI, RobotConstants.TRANSLATION_PID.kD);
    rotController = new PIDController(RobotConstants.ROTATION_PID.kP, RobotConstants.ROTATION_PID.kI, RobotConstants.ROTATION_PID.kD);
    rotController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    TRAngle = Math.atan2(chassis.getFusedPose().getY() - targetPose.getY(), chassis.getFusedPose().getX() - targetPose.getX());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // This first bit basically calculates polar coordinates for the robot with the target as the origin
    // Change target orbit distance based on joystick input
    orbitDistance = Constants.ORBIT_RADIUS + (ySupplier.getAsDouble() * Constants.ORBIT_RADIUS_MARGIN);
    TRAngle = TRAngle + (Math.asin((xSupplier.getAsDouble()) / orbitDistance) * RobotConstants.MAX_SPEED * Util.triggerAdjust(lTriggerSupplier.getAsDouble(), rTriggerSupplier.getAsDouble()));

    // Then converts the polar coordinates to field coordinates
    calcTargetPose();

    // Then uses PID to move the robot to the target pose
    xController.setSetpoint(xTarget);
    yController.setSetpoint(yTarget);
    rotController.setSetpoint(rotTarget);

    chassis.setChassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
      xController.calculate(chassis.getFusedPose().getX()),
      yController.calculate(chassis.getFusedPose().getY()),
      rotController.calculate(chassis.getFusedPose().getRotation().getRadians() % (2 * Math.PI))), // Not sure if I have to make sure the angle is in the range [0, 2 * PI)
      chassis.getFusedPose().getRotation()
    );
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  public void calcTargetPose() {
    xTarget = Math.cos(TRAngle) * orbitDistance;
    yTarget = Math.sin(TRAngle) * orbitDistance;

    xTarget += targetPose.getX();
    yTarget += targetPose.getY();

    rotTarget = TRAngle + Math.PI; // Offset by 180 degrees to get robot-target angle as this is the angle the robot will be facing
    rotTarget = rotTarget % (2 * Math.PI); // Wrap angle to [0, 2 * PI)
  }
}