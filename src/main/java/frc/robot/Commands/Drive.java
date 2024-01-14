// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Systems.Chassis.Chassis;

/** This is the command for teleoperation of the chassis */
public class Drive extends Command{
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

    public Drive(
      Chassis chassis,
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier,
      DoubleSupplier lTriggerSupplier, DoubleSupplier rTriggerSupplier,
      BooleanSupplier upSupplier, BooleanSupplier rightSupplier, BooleanSupplier downSupplier, BooleanSupplier leftSupplier) {
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
    // Maps the Y, B, A, X buttons to create a vector and then gets the direction of the vector using trigonometry,
    // then fits it to the range [0, 2 * PI)
    //double x = (upSupplier.getAsBoolean() ? 1 : 0) - (downSupplier.getAsBoolean() ? 1 : 0);
    //double y = (rightSupplier.getAsBoolean() ? 1 : 0) - (leftSupplier.getAsBoolean() ? 1 : 0);
   // double dir = Math.atan2(y, x);
    //dir = dir < 0 ? dir + 2 * Math.PI : dir;

    //thetaPID.setSetpoint(dir * -1);

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      triggerAdjust(modifyJoystick(-xSupplier.getAsDouble())) * RobotConstants.MAX_SPEED,
      triggerAdjust(modifyJoystick(-ySupplier.getAsDouble())) * RobotConstants.MAX_SPEED,
      triggerAdjust(modifyJoystick(-rotSupplier.getAsDouble())) * RobotConstants.MAX_ANGULAR_VELOCITY,
      chassis.getFusedPose().getRotation() //TODO will have to change to be fused pose instead of gyro
      );

    // ChassisSpeeds speeds = new ChassisSpeeds(
    //   triggerAdjust(modifyJoystick(-xSupplier.getAsDouble())) * RobotConstants.MAX_SPEED,
    //   triggerAdjust(modifyJoystick(-ySupplier.getAsDouble())) * RobotConstants.MAX_SPEED,
    //   triggerAdjust(modifyJoystick(-rotSupplier.getAsDouble())) * RobotConstants.MAX_ANGULAR_VELOCITY
    // );

   // double currentRot = chassis.getFusedPose().getRotation().getRadians() % (Math.PI * 2);
  //  double dpadSpeed =
     // upSupplier.getAsBoolean() || rightSupplier.getAsBoolean() || downSupplier.getAsBoolean() || leftSupplier.getAsBoolean()
      //? thetaPID.calculate(currentRot) : 0;
   // speeds = new ChassisSpeeds(
     // xLimiter.calculate(speeds.vxMetersPerSecond),
      //yLimiter.calculate(speeds.vyMetersPerSecond),
     // thetaLimiter.calculate(speeds.omegaRadiansPerSecond) + dpadSpeed);
     
    chassis.chassisSpeed = speeds;
    chassis.convertToStates(); 
    chassis.drive();
  }

  @Override
  public void end(boolean interrupted) {
    chassis.chassisSpeed = new ChassisSpeeds(0.0, 0.0, 0.0);
  }

  /**
   * Adjusts the speeds of the given input depending on trigger input, with left
   * trigger decreasing speed and RT increasing
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
    double triggers = (1 - upAdjust) + (deadband(rTriggerSupplier.getAsDouble(), Constants.TRIGGER_DEADBAND) * upAdjust)
        - (deadband(lTriggerSupplier.getAsDouble(), Constants.TRIGGER_DEADBAND) * downAdjust);
    return in * triggers;
  }
    /**
   * Applies a deadband to the given joystick axis value
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
   * @param value
   * @return
   */
  private static double modifyJoystick(double value) {
    // Deadband
    value = deadband(value, Constants.JOYSTICK_DEADBAND);
    // Square the axis
    // value = Math.copySign(value * value, value);
    return value;
  }
}