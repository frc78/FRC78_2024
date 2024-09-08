package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.CommandSwerveDrivetrain;
import java.util.function.Supplier;

public class AlignToNote extends Command {

  private final CommandSwerveDrivetrain chassis;
  private Supplier<ChassisSpeeds> speedsSupplier;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Fisheye");
  NetworkTableEntry yaw = table.getEntry("targetYaw");

  private final PIDController rotationController = new PIDController(0.15, 0, 0.0015);
  private final SwerveRequest.ApplyChassisSpeeds applyChassisSpeeds =
      new SwerveRequest.ApplyChassisSpeeds();

  /** Creates a new FieldOrientedDrive. */
  public AlignToNote(CommandSwerveDrivetrain chassis, Supplier<ChassisSpeeds> speedsSupplier) {
    this.chassis = chassis;
    this.speedsSupplier = speedsSupplier;

    rotationController.setSetpoint(0.0);

    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    rotationController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double finalYaw = yaw.getDouble(0.0);

    double rotation = rotationController.calculate(finalYaw);

    ChassisSpeeds align = speedsSupplier.get();

    align.omegaRadiansPerSecond = rotation;

    chassis.setControl(applyChassisSpeeds.withSpeeds(align));
  }
}
