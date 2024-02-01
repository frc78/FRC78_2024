// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.classes.Util;
import frc.robot.constants.Constants;
import frc.robot.subsystems.chassis.Chassis;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

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
  private final ProfiledPIDController thetaPID;
  private final double maxSpeed;
  private final double maxAngularVelocity;

  public Drive(
      Chassis chassis,
      XboxController controller,
      double maxSpeed,
      double maxAngularVelocity,
      PIDConstants cardinalPidConstants,
      double translationRateLimit,
      double rotationRateLimit,
      Constraints constraints) {
    this.chassis = chassis;
    this.xSupplier = () -> controller.getLeftY();
    this.ySupplier = () -> controller.getLeftX();
    this.rotSupplier = () -> controller.getRightX();
    this.lTriggerSupplier = () -> controller.getLeftTriggerAxis();
    this.rTriggerSupplier = () -> controller.getRightTriggerAxis();
    this.upSupplier = () -> controller.getYButton();
    this.rightSupplier = () -> controller.getBButton();
    this.downSupplier = () -> controller.getAButton();
    this.leftSupplier = () -> controller.getXButton();
    this.maxSpeed = maxSpeed;
    this.maxAngularVelocity = maxAngularVelocity;

    xLimiter = new SlewRateLimiter(translationRateLimit, -translationRateLimit, 0);
    yLimiter = new SlewRateLimiter(translationRateLimit, -translationRateLimit, 0);
    thetaLimiter = new SlewRateLimiter(rotationRateLimit, -rotationRateLimit, 0);

    thetaPID =
        new ProfiledPIDController(
            cardinalPidConstants.kP,
            cardinalPidConstants.kI,
            cardinalPidConstants.kD,
            constraints); // TODO tune
    thetaPID.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(chassis);
  }

  @Override
  public void execute() {
    double triggerAdjust =
        Util.triggerAdjust(
            MathUtil.applyDeadband(lTriggerSupplier.getAsDouble(), Constants.TRIGGER_DEADBAND),
            MathUtil.applyDeadband(rTriggerSupplier.getAsDouble(), Constants.TRIGGER_DEADBAND));
    double x = Util.modifyJoystick(-xSupplier.getAsDouble()) * triggerAdjust;
    double y = Util.modifyJoystick(-ySupplier.getAsDouble()) * triggerAdjust;
    double rot = Util.modifyJoystick(-rotSupplier.getAsDouble()) * triggerAdjust;
    SmartDashboard.putNumber("triggerAdjust", triggerAdjust);

    // Maps the Y, B, A, X buttons to create a vector and then gets the direction of the vector
    // using trigonometry,
    // then fits it to the range [0, 2 * PI)
    double xCardinal = (upSupplier.getAsBoolean() ? 1 : 0) - (downSupplier.getAsBoolean() ? 1 : 0);
    double yCardinal =
        (rightSupplier.getAsBoolean() ? 1 : 0) - (leftSupplier.getAsBoolean() ? 1 : 0);
    double dir = -Math.atan2(yCardinal, xCardinal);
    dir = dir < 0 ? dir + 2 * Math.PI : dir;
    Logger.recordOutput("goalCardinal", dir);

    thetaPID.setGoal(dir);

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            x * maxSpeed,
            y * maxSpeed,
            rot * maxAngularVelocity,
            chassis.getFusedPose().getRotation());

    double dpadSpeed =
        upSupplier.getAsBoolean()
                || rightSupplier.getAsBoolean()
                || downSupplier.getAsBoolean()
                || leftSupplier.getAsBoolean()
            ? thetaPID.calculate(chassis.getFusedPose().getRotation().getRadians())
            : 0;
    speeds =
        new ChassisSpeeds(
            xLimiter.calculate(speeds.vxMetersPerSecond),
            yLimiter.calculate(speeds.vyMetersPerSecond),
            thetaLimiter.calculate(speeds.omegaRadiansPerSecond) + dpadSpeed);

    chassis.setChassisSpeed = speeds;
    chassis.convertToStates();
    chassis.drive();
  }

  @Override
  public void end(boolean interrupted) {
    chassis.setChassisSpeed = new ChassisSpeeds(0.0, 0.0, 0.0);
  }
}
