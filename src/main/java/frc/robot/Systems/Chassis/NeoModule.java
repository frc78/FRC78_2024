// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Systems.Chassis;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Classes.ModuleConfig;
import frc.robot.Constants.RobotConstants;

/**
 * Neo implementation of SwerveModule
 */
public class NeoModule implements SwerveModule {
    protected ModuleConfig config;

    protected CANSparkMax drive;
    protected CANSparkMax steer;

    protected SparkPIDController drivePID;
    protected SparkPIDController steerPID;
    private RelativeEncoder driveEnc;
    private AbsoluteEncoder steerEnc;

    // this is set but never used
    private SwerveModuleState desiredState;

    NeoModule(ModuleConfig config) {
        this.config = config;
        // drive = new Spark(this.config.driveID);
        // steer = new Spark(this.config.steerID);

        drive = new CANSparkMax(this.config.driveID, MotorType.kBrushless);
        steer = new CANSparkMax(this.config.steerID, MotorType.kBrushless);

        driveEnc = drive.getEncoder();
        steerEnc = steer.getAbsoluteEncoder(Type.kDutyCycle);
        drivePID = drive.getPIDController();
        steerPID = steer.getPIDController();

        desiredState = new SwerveModuleState();

        initialize();
    }

    /**
     * Initializes the NEO module. This should be called in the constructor of the module class.
     */
    @Override
    public void initialize() {
        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out
        // TODO I don't think we should restore factory defaults each time. It could deplete the life of the spark maxes faster than necessary.
        //  We can use the rev client to reset if we have to swap a controller
        drive.restoreFactoryDefaults();
        steer.restoreFactoryDefaults();
        drivePID.setFeedbackDevice(driveEnc); // This doesn't do anything
        steerPID.setFeedbackDevice(steerEnc);

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        driveEnc.setPositionConversionFactor(RobotConstants.DRIVE_ENC_TO_METERS);
        driveEnc.setVelocityConversionFactor(RobotConstants.DRIVE_ENC_VEL_TO_METERS);


        // WPILib swerve works in rotations, which is the native units for the encoders.
        // Also, meters is definitely the wrong unit here
        steerEnc.setPositionConversionFactor(RobotConstants.STEER_ENC_POS_TO_METERS); // TODO have to change this to be the encoder without the steering gear ratio
        steerEnc.setVelocityConversionFactor(RobotConstants.STEER_ENC_VEL_TO_METERS);

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        // The motor directions shouldn't affect the invertedness. What matters is does it follow the coordinate system of the wpilib kinematics.
        // Wpilib wants counter clockwise rotation to be positive. You have to measure this empirically
        steerEnc.setInverted(RobotConstants.STEER_ENC_INVERTED);
        // AFTER making sure the steer encoder is correctly inverted, invert the steer motor if positive voltage decreases encoder value
        steer.setInverted(RobotConstants.STEER_INVERTED);
        // Drive motor should be inverted if the encoder is at 0 (straight ahead) and positive voltage results in wheel spinning backwards
        drive.setInverted(RobotConstants.DRIVE_INVERTED);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        steerPID.setPositionPIDWrappingEnabled(true);
        steerPID.setPositionPIDWrappingMinInput(RobotConstants.STEER_ENC_PID_MIN);
        steerPID.setPositionPIDWrappingMaxInput(RobotConstants.STEER_ENC_PID_MAX);

        // I don't wanna set these in code becuase, like we saw, each motor behaves slightly differently. What we _can_ do is download the configuration files
        // for each module and save them so that if we need to hot-swap a motor controller we can.
        // Set the PID gains for the driving motor
        drivePID.setP(RobotConstants.K_DRIVE_P);
        drivePID.setI(RobotConstants.K_DRIVE_I);
        drivePID.setD(RobotConstants.K_DRIVE_D);
        drivePID.setFF(RobotConstants.K_DRIVE_FF);
        drivePID.setOutputRange(RobotConstants.DRIVE_OUT_MIN, RobotConstants.DRIVE_OUT_MAX);

        // Same thing. Each wheel will be slightly different. We can have different constants for each wheel in the code, and pass them into this class' constructor
        // Set the PID gains for the turning motor
        steerPID.setP(RobotConstants.K_STEER_P);
        steerPID.setI(RobotConstants.K_STEER_I);
        steerPID.setD(RobotConstants.K_STEER_D);
        steerPID.setFF(RobotConstants.K_STEER_FF);
        steerPID.setOutputRange(RobotConstants.STEER_OUT_MIN, RobotConstants.STEER_OUT_MAX);

        drive.setSmartCurrentLimit(RobotConstants.DRIVE_CURRENT_LIMIT);
        steer.setSmartCurrentLimit(RobotConstants.STEER_CURRENT_LIMIT);

        drive.setIdleMode(RobotConstants.DRIVE_IDLE);
        steer.setIdleMode(RobotConstants.STEER_IDLE);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        // Since this might deplete the life of the spark, instead we can check the 'isReset' error code
        // drive.getFault(CANSparkBase.FaultID.kHasReset)
        drive.burnFlash();
        steer.burnFlash();

        desiredState.angle = new Rotation2d(steerEnc.getPosition());
        driveEnc.setPosition(0);
    }

    /**
     * Resets the drive encoder to 0 meters
     */
    @Override
    public void resetEncoders() {
        driveEnc.setPosition(0);
    }

    /**
     * @return The velocity of the drive encoder, in meters per second
     */
    @Override
    public double getDriveVelocity() {
        return driveEnc.getVelocity();
    }

    /**
     * @return The relative position of the drive encoder, in meters
     */
    @Override
    public double getDrivePosition() {
        return driveEnc.getPosition();
    }

    /**
     * @return The absolute position of the drive encoder, as a {@link Rotation2d#Rotation2d Rotation2d}
     */
    @Override
    public Rotation2d getSteerPosition() {
        return Rotation2d.fromRadians(steerEnc.getPosition());
    }

    /**
     * @return The state of the module as a {@link SwerveModuleState#SwerveModuleState SwerveModuleState}
     */
    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getSteerPosition());
    }

    /**
     * @return The position of the module as a {@link SwerveModulePosition#SwerveModulePosition SwerveModulePosition}
     */
    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getSteerPosition());
    }

    /**
     * Sets the desired velocity of the module. Should only be used if you want to ONLY set the velocity. If not, then use {@link #setDesiredState(SwerveModuleState)}
     *
     * @param veloctiy The desired velocity of the module, in meters per second
     */
    @Override
    public void setVelocity(double velocity) {
        drivePID.setReference(velocity, CANSparkMax.ControlType.kVelocity);

        desiredState.speedMetersPerSecond = velocity;
    }

    /**
     * Sets the desired rotation of the module. Should only be used if you want to ONLY set the rotation. If not, then use {@link #setDesiredState(SwerveModuleState)}
     *
     * @param rotation The desired rotation of the module
     */
    // unused method
    @Override
    public void setRotation(Rotation2d rotation) {
        Rotation2d correctedRot = rotation.plus(Rotation2d.fromRadians(config.offset));
        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState correctedState = SwerveModuleState.optimize(new SwerveModuleState(0, correctedRot), getSteerPosition());
        steerPID.setReference(correctedState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
        desiredState.angle = rotation;
    }

    /**
     * Sets the desired state of the module. Should be used if you want to set both the velocity and rotation at the same time
     *
     * @param state The desired state of the module
     */
    @Override
    public void setState(SwerveModuleState state) {
        SwerveModuleState correctedRot = new SwerveModuleState();
        //Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getSteerPosition());

        //Sets the PID goals to the desired states
        drivePID.setReference(optimizedState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        steerPID.setReference(optimizedState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        desiredState = state;
        SmartDashboard.putNumber(config.driveID + " setting rot", optimizedState.angle.getRadians());//Changed this to divide by 2 pi and ad o.5 to map the joystick input (-pi to pi) to a zero to 1
        SmartDashboard.putNumber(config.driveID + " getting rot", steerEnc.getPosition() - Math.PI);
        SmartDashboard.putNumber(config.driveID + "getting speed", getDriveVelocity());
        SmartDashboard.putNumber(config.driveID + "setting speed", optimizedState.speedMetersPerSecond);
    }
}