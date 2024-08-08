package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.classes.ModuleConfig;
import frc.robot.classes.Structs.FFConstants;

public class FalconModule implements SwerveModule{
    private Rotation2d angleOffset;

    private TalonFX steer;
    private TalonFX drive;
    private CANcoder encoder;

    private final SimpleMotorFeedforward feedforward;

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public FalconModule(int driveID, int steerID, int encoderID, ModuleConfig config, FFConstants ffConstants){
        this.angleOffset = config.offset;
        
        /* Angle Encoder Config */
        encoder = new CANcoder(encoderID);
        encoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        steer = new TalonFX(steerID);
        steer.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        drive = new TalonFX(driveID);
        drive.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        drive.getConfigurator().setPosition(0.0);

        feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        steer.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            drive.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = feedforward.calculate(desiredState.speedMetersPerSecond);
            drive.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        steer.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(drive.getVelocity().getValue(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(steer.getPosition().getValue())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(drive.getPosition().getValue(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(steer.getPosition().getValue())
        );
    }
}