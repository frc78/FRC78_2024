// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.classes.Util;
import frc.robot.constants.Constants;
import frc.robot.subsystems.chassis.Chassis;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** This is the command for teleoperation of the chassis */
public class Drive extends Command {
  private Chassis chassis;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier rotSupplier;
  private final DoubleSupplier lTriggerSupplier;
  private final DoubleSupplier rTriggerSupplier;
  private final BooleanSupplier upSupplier, rightSupplier, downSupplier, leftSupplier;

  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter thetaLimiter;
  private final PIDController thetaPID;
  private final double maxSpeed;
  private final double maxAngularVelocity;

  public Drive(
      Chassis chassis,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotSupplier,
      DoubleSupplier lTriggerSupplier,
      DoubleSupplier rTriggerSupplier,
      BooleanSupplier upSupplier,
      BooleanSupplier rightSupplier,
      BooleanSupplier downSupplier,
      BooleanSupplier leftSupplier,
      double maxSpeed,
      double maxAngularVelocity) {
    this.chassis = chassis;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotSupplier = rotSupplier;
    this.lTriggerSupplier = lTriggerSupplier;
    this.rTriggerSupplier = rTriggerSupplier;
    this.upSupplier = upSupplier;
    this.rightSupplier = rightSupplier;
    this.downSupplier = downSupplier;
    this.leftSupplier = leftSupplier;
    this.maxSpeed = maxSpeed;
    this.maxAngularVelocity = maxAngularVelocity;

    xLimiter = new SlewRateLimiter(11, -11, 0);
    yLimiter = new SlewRateLimiter(11, -11, 0);
    thetaLimiter = new SlewRateLimiter(30, -30, 0);

    thetaPID = new PIDController(2.5, 7, 0.16); // TODO almost perfect
    // thetaPID = new PIDController(5, 0, 0.025); good for 90 turns
    // thetaPID = new PIDController(5, 0, 0);
    thetaPID.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(chassis);
  }

  @Override
  public void execute() {
    // Maps the Y, B, A, X buttons to create a vector and then gets the direction of the vector
    // using trigonometry,
    // then fits it to the range [0, 2 * PI)
    // double x = (upSupplier.getAsBoolean() ? 1 : 0) - (downSupplier.getAsBoolean() ? 1 : 0);
    // double y = (rightSupplier.getAsBoolean() ? 1 : 0) - (leftSupplier.getAsBoolean() ? 1 : 0);
    // double dir = Math.atan2(y, x);
    // dir = dir < 0 ? dir + 2 * Math.PI : dir;

    // thetaPID.setSetpoint(dir * -1);

    // Processed inputs
    double triggerAdjust =
        Util.triggerAdjust(
            Util.deadband(lTriggerSupplier.getAsDouble(), Constants.TRIGGER_DEADBAND),
            Util.deadband(rTriggerSupplier.getAsDouble(), Constants.TRIGGER_DEADBAND));
    double x = Util.modifyJoystick(-xSupplier.getAsDouble()) * triggerAdjust;
    double y = Util.modifyJoystick(-ySupplier.getAsDouble()) * triggerAdjust;
    double rot = Util.modifyJoystick(-rotSupplier.getAsDouble()) * triggerAdjust;
    SmartDashboard.putNumber("triggerAdjust", triggerAdjust);

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            x * maxSpeed,
            y * maxSpeed,
            rot * maxAngularVelocity,
            chassis.getFusedPose().getRotation());

    // ChassisSpeeds speeds = new ChassisSpeeds(
    //   triggerAdjust(modifyJoystick(-xSupplier.getAsDouble())) * RobotConstants.MAX_SPEED,
    //   triggerAdjust(modifyJoystick(-ySupplier.getAsDouble())) * RobotConstants.MAX_SPEED,
    //   triggerAdjust(modifyJoystick(-rotSupplier.getAsDouble())) *
    // RobotConstants.MAX_ANGULAR_VELOCITY
    // );

    // double currentRot = chassis.getFusedPose().getRotation().getRadians() % (Math.PI * 2);
    //  double dpadSpeed =
    // upSupplier.getAsBoolean() || rightSupplier.getAsBoolean() || downSupplier.getAsBoolean() ||
    // leftSupplier.getAsBoolean()
    // ? thetaPID.calculate(currentRot) : 0;
    // speeds = new ChassisSpeeds(
    // xLimiter.calculate(speeds.vxMetersPerSecond),
    // yLimiter.calculate(speeds.vyMetersPerSecond),
    // thetaLimiter.calculate(speeds.omegaRadiansPerSecond) + dpadSpeed);

    chassis.setChassisSpeed = speeds;
    chassis.convertToStates();
    chassis.drive();
  }

  @Override
  public void end(boolean interrupted) {
    chassis.setChassisSpeed = new ChassisSpeeds(0.0, 0.0, 0.0);
  }
}
