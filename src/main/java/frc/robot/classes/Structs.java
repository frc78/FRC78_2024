// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

/** Class for structs */
public class Structs {

  /** Struct for feed forward constants. Contains 3 doubles of kS, kV, and kA */
  public static class FFConstants {
    public final double kS, kV, kA, kFF;

    public FFConstants(double kS, double kV, double kA, double kFF) {
      this.kS = kS;
      this.kV = kV;
      this.kA = kA;
      this.kFF = kFF;
    }

    public FFConstants(double kS, double kV, double kA) {
      this(kS, kV, kA, 0);
    }
  }

  public static class RateLimits {
    public final double translationRateLimit, rotationRateLimit;

    public RateLimits(double translationRateLimit, double rotationRateLimit) {
      this.translationRateLimit = translationRateLimit;
      this.rotationRateLimit = rotationRateLimit;
    }
  }

  public static class MotionLimits {
    public final double maxSpeed, maxAcceleration, maxAngularSpeed, maxAngularAcceleration;

    public MotionLimits(
        double maxSpeed,
        double maxAcceleration,
        double maxAngularSpeed,
        double maxAngularAcceleration) {
      this.maxSpeed = maxSpeed;
      this.maxAcceleration = maxAcceleration;
      this.maxAngularSpeed = maxAngularSpeed;
      this.maxAngularAcceleration = maxAngularAcceleration;
    }
  }

  public static class Range {
    public double min, max;

    public Range(double min, double max) {
      this.min = min;
      this.max = max;
    }

    public double getRange() {
      return max - min;
    }
  }
}
