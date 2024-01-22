// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
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
    thetaPID.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(chassis);
  }

  @Override
  public void execute() {

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            triggerAdjust(modifyJoystick(-xSupplier.getAsDouble())) * maxSpeed,
            triggerAdjust(modifyJoystick(-ySupplier.getAsDouble())) * maxSpeed,
            triggerAdjust(modifyJoystick(-rotSupplier.getAsDouble())) * maxAngularVelocity,
            chassis
                .getFusedPose()
                .getRotation() // TODO will have to change to be fused pose instead of gyro
            );

    chassis.chassisSpeed = speeds;
    chassis.convertToStates();
    chassis.drive();
  }

  @Override
  public void end(boolean interrupted) {
    chassis.chassisSpeed = new ChassisSpeeds(0.0, 0.0, 0.0);
  }

  /**
   * Adjusts the speeds of the given input depending on trigger input, with left trigger decreasing
   * speed and RT increasing
   *
   * @param in
   * @return Adjusted speed
   */
  public double triggerAdjust(double in) {
    double upAdjust = 0.5;
    double downAdjust = 0.25;
    // Default speed = 1 - upAdjust
    // Full left trigger = 1 - upAdjust - downAdjust
    // Full right trigger = 1
    double triggers =
        (1 - upAdjust)
            + (deadband(rTriggerSupplier.getAsDouble(), Constants.TRIGGER_DEADBAND) * upAdjust)
            - (deadband(lTriggerSupplier.getAsDouble(), Constants.TRIGGER_DEADBAND) * downAdjust);
    return in * triggers;
  }

  /**
   * Applies a deadband to the given joystick axis value
   *
   * @param value
   * @param deadband
   * @return
   */
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      return (value > 0.0 ? value - deadband : value + deadband) / (1.0 - deadband);
    } else {
      return 0.0;
    }
  }

  /**
   * Processes the given joystick axis value, applying deadband and squaring it
   *
   * @param value
   * @return
   */
  private static double modifyJoystick(double value) {
    // Deadband
    value = deadband(value, Constants.JOYSTICK_DEADBAND);
    return Math.pow(value, 3);
  }
}
