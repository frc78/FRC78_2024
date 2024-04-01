package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;
import java.util.function.Supplier;

public class AlignToNote extends Command {
  private final Chassis chassis;
  private ChassisSpeeds speeds;
  private Supplier<ChassisSpeeds> speedsSupplier;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Fisheye");
  NetworkTableEntry yaw = table.getEntry("targetYaw");

  // private final PIDController translationController = new PIDController(0.07, 0, 0);
  private final PIDController rotationController = new PIDController(0.15, 0, 0.0015);

  /** Creates a new FieldOrientedDrive. */
  public AlignToNote(Chassis chassis, Supplier<ChassisSpeeds> speedsSupplier) {
    this.chassis = chassis;
    this.speeds = speedsSupplier.get();
    this.speedsSupplier = speedsSupplier;

    // translationController.setSetpoint(0.0);
    rotationController.setSetpoint(0.0);

    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    // translationController.reset();
    rotationController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double finalYaw = yaw.getDouble(0.0);

    double rotation = rotationController.calculate(finalYaw);

    ChassisSpeeds align = speedsSupplier.get();

    align.omegaRadiansPerSecond = rotation;

    chassis.driveRobotRelative(
      align);
  }
}
