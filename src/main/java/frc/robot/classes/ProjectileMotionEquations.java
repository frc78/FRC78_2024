package frc.robot.classes;

public class ProjectileMotionEquations {
  public static double calculateLaunchVelocity(
      double distance, double finalHeight, double launchAngle) {

    double nominator = Math.pow(distance, 2) * 9.81;
    double denominator =
        distance * Math.sin(2 * launchAngle) - 2 * finalHeight * Math.pow(Math.cos(launchAngle), 2);
    return Math.sqrt(nominator / denominator);
  }
}
