package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;

public class DriveToAprilTag extends Command {
  private final Chassis chassis;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry tv = table.getEntry("tv");

  private final PIDController translationControllerX = new PIDController(0.03, 0, 0);
  private final PIDController translationControllerY = new PIDController(0.03, 0, 0);

  /** Creates a new FieldOrientedDrive. */
  public DriveToAprilTag(Chassis chassis) {
    this.chassis = chassis;

    translationControllerX.setSetpoint(0.0);
    translationControllerY.setSetpoint(0.0);

    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    translationControllerX.reset();
    translationControllerY.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double dx = tx.getDouble(0.0);
    double dy = ty.getDouble(0.0);

    double translationX = translationControllerX.calculate(dx);
    double translationY = translationControllerY.calculate(dy);

    chassis.driveRobotRelative(new ChassisSpeeds(translationY, translationX, 0));
  }

  @Override
  public boolean isFinished() {
    return ((Math.abs(tx.getDouble(100)) < 5) && (Math.abs(ty.getDouble(100)) < .5) && (tv.getDouble(0) == 1));
  }

  @Override
  public void end(boolean interrupted) {
    chassis.driveRobotRelative(new ChassisSpeeds());
  }
}
