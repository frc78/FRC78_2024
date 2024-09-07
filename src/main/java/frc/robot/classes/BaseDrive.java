// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import java.util.function.DoubleSupplier;

/**
 * Base driving class, takes in joysticks and performs all processing, outputting chassis speeds
 * that can then be fed into further processing
 */
public class BaseDrive {
  public Structs.MotionLimits motionLimits;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier rotSupplier;
  private final DoubleSupplier lTriggerSupplier;
  private final DoubleSupplier rTriggerSupplier;

  // private final SlewRateLimiter xLimiter;
  // private final SlewRateLimiter yLimiter;
  // private final SlewRateLimiter thetaLimiter;

  public BaseDrive(
      XboxController controller, Structs.MotionLimits motionLimits, Structs.RateLimits rateLimits) {
    this.motionLimits = motionLimits;
    this.xSupplier = () -> -controller.getLeftY();
    this.ySupplier = () -> -controller.getLeftX();
    this.rotSupplier = () -> -controller.getRightX();
    this.lTriggerSupplier = controller::getLeftTriggerAxis;
    this.rTriggerSupplier = controller::getRightTriggerAxis;

    // xLimiter = new SlewRateLimiter(rateLimits.translationRateLimit);
    // yLimiter = new SlewRateLimiter(rateLimits.translationRateLimit);
    // thetaLimiter = new SlewRateLimiter(rateLimits.rotationRateLimit);
  }

  public ChassisSpeeds calculateChassisSpeeds() {
    double trigger =
        Util.triggerAdjust(
            Util.modifyTrigger(lTriggerSupplier.getAsDouble()),
            Util.modifyTrigger(rTriggerSupplier.getAsDouble()));

    double speed = trigger * motionLimits.maxSpeed;
    double angularSpeed = trigger * motionLimits.maxAngularSpeed;

    double x = Util.modifyJoystick(xSupplier.getAsDouble()) * speed;
    double y = Util.modifyJoystick(ySupplier.getAsDouble()) * speed;
    double rot = Util.modifyJoystick(rotSupplier.getAsDouble()) * angularSpeed;

    return new ChassisSpeeds(x, y, rot);
  }
}
