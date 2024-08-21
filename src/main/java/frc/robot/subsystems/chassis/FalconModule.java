package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.lib.math.Conversions;
import frc.robot.classes.ModuleConfigFalcon;
import frc.robot.classes.Structs.FFConstants;

public class FalconModule implements SwerveModule {
  private double angleOffset;

  private TalonFX steer;
  private TalonFX drive;
  private CANcoder encoder;

  private final SimpleMotorFeedforward feedforward;

  /* drive motor control requests */
  private final DutyCycleOut driveDutyCycle;
  private final VelocityVoltage driveVelocity;

  /* angle motor control requests */
  private final PositionVoltage anglePosition;
  private final ModuleConfigFalcon config;

  public FalconModule(
      int driveID,
      int steerID,
      int encoderID,
      double angleOffset,
      ModuleConfigFalcon config,
      FFConstants ffConstants) {
    this.config = config;
    this.angleOffset = angleOffset;

    /* Angle Encoder Config */
    encoder = new CANcoder(encoderID);
    // MagnetSensorConfigs encoderConfig = new MagnetSensorConfigs(); // TODO will need this later!
    // encoderConfig.MagnetOffset = angleOffset;
    // encoderConfig.SensorDirection =
    //     config.steerEncoderInverted
    //         ? SensorDirectionValue.Clockwise_Positive
    //         : SensorDirectionValue.CounterClockwise_Positive; // Migjt jave TOBEB CROACHCC BESS
    // encoderConfig.AbsoluteSensorRange =
    //     AbsoluteSensorRangeValue.Unsigned_0To1; // THIS also a bit, deserves a drink!

    /* Angle Motor Config */
    steer = new TalonFX(steerID);

    /* Drive Motor Config */
    drive = new TalonFX(driveID);

    anglePosition = new PositionVoltage(0);
    driveDutyCycle = new DutyCycleOut(0);
    driveVelocity = new VelocityVoltage(0);

    feedforward = new SimpleMotorFeedforward(ffConstants.kS, ffConstants.kV, ffConstants.kA);

    initialize();
  }

  @Override
  public void setState(SwerveModuleState desiredState) {
    Boolean isOpenLoop = true; // TODO For fun
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    steer.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
    setSpeed(desiredState, isOpenLoop);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      driveDutyCycle.Output = desiredState.speedMetersPerSecond / config.maxDriveSpeed;
      drive.setControl(driveDutyCycle);
    } else {
      driveVelocity.Velocity = desiredState.speedMetersPerSecond * config.driveMPS_RPS;
      driveVelocity.FeedForward = feedforward.calculate(desiredState.speedMetersPerSecond);
      drive.setControl(driveVelocity);
    }
  }

  public Rotation2d getCANcoder() {
    return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValue());
  }

  public void resetToAbsolute() {
    double absolutePosition = getCANcoder().getRotations() - angleOffset;
    steer.setPosition(absolutePosition);
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getSteerPosition());
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Conversions.rotationsToMeters(
            drive.getPosition().getValue(), Constants.Swerve.wheelCircumference),
        Rotation2d.fromRotations(steer.getPosition().getValue()));
  }

  @Override
  public void initialize() {
    encoder.getConfigurator().apply(this.config.steerEncoderConfig);

    steer.getConfigurator().apply(this.config.steerMotorConfig);
    resetToAbsolute();

    drive.getConfigurator().apply(this.config.driveMotorConfig);
    drive.getConfigurator().setPosition(0.0);
  }

  @Override
  public void setBrake(Boolean y) {
    drive.setNeutralMode(y ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public double getDriveVelocity() {
    return drive.getVelocity().getValue() / config.driveMPS_RPS;
  }

  @Override
  public double getDrivePosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDrivePosition'");
  }

  @Override
  public Rotation2d getSteerPosition() {
    return Rotation2d.fromRotations(steer.getPosition().getValue());
  }

  @Override
  public SwerveModuleState getOptimizedState() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getOptimizedState'");
  }

  @Override
  public SwerveModuleState getRealState() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getRealState'");
  }

  @Override
  public void openLoopDiffDrive(double voltage) {
    throw new UnsupportedOperationException("Unimplemented method 'openLoopDiffDrive'");
  }

  @Override
  public void logMotor(SysIdRoutineLog log) {
    throw new UnsupportedOperationException("Unimplemented method 'logMotor'");
  }

  @Override
  public void enableBrakeMode() {
    setBrake(true);
  }

  @Override
  public void enableCoastMode() {
    setBrake(false);
  }
}
