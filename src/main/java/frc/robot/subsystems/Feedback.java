// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feedback extends SubsystemBase {
  /** Creates a new Feedback. */
  private CANdle bracelet;

  public Feedback(int candleID) {
    bracelet = new CANdle(candleID, "CANivore");
    CANdleConfiguration config = new CANdleConfiguration();
    config.brightnessScalar = 0.5; // dim the LEDs to half brightness
    bracelet.configAllSettings(config);
    bracelet.configLEDType(LEDStripType.RGB);
    this.off();
    this.setDefaultCommand(this.setAllianceColor());
  }

  public Command setAllianceColor() {
    return this.run(
            () -> {
              var a = DriverStation.getAlliance();
              if (a.isPresent() && a.get().equals(Alliance.Red)) {
                setMulti(Color.kRed);
              } else {
                setMulti(Color.kBlue);
              }
            })
        .withName("SetAllianceColor");
  }

  public Command shooterWheelsAtSpeed() {
    return this.startEnd(
            () -> {
              StrobeAnimation sa = new StrobeAnimation(0, 255, 0, 255, 0.2, 68, 0);
              bracelet.clearAnimation(1);
              bracelet.clearAnimation(2);
              bracelet.clearAnimation(3);
              bracelet.clearAnimation(4);
              bracelet.animate(sa, 1);
            },
            this::off)
        .withName("ShooterWheelsAtSpeed");
  }

  public Command noteInCartridge() {
    return this.startEnd(
            () -> {
              bracelet.clearAnimation(1);
              bracelet.clearAnimation(2);
              bracelet.clearAnimation(3);
              bracelet.clearAnimation(4);
              bracelet.setLEDs(0, 255, 0);
            },
            this::off)
        .withName("noteInCartridge");
  }

  public Command rainbows() {
    return this.startEnd(this::animate, this::off);
  }

  public Command intakeCurrentSpike() {
    return this.startEnd(
            () -> {
              bracelet.clearAnimation(1);
              bracelet.clearAnimation(2);
              bracelet.clearAnimation(3);
              bracelet.clearAnimation(4);
              bracelet.setLEDs(255, 255, 0);
            },
            this::off)
        .withName("IntakeCurrentSpike");
  }

  public Command turnOffLEDs() {
    return this.startEnd(
            () -> {
              bracelet.clearAnimation(1);
              bracelet.clearAnimation(2);
              bracelet.clearAnimation(3);
              bracelet.clearAnimation(4);
              bracelet.setLEDs(0, 0, 0);
            },
            this::off)
        .withName("TurnOffLEDs");
  }

  public void disabledColorPattern() {
    ColorFlowAnimation cfa = new ColorFlowAnimation(0, 0, 255, 255, 0.1, 15, Direction.Forward, 7);
    bracelet.clearAnimation(1);
    bracelet.clearAnimation(2);
    bracelet.clearAnimation(3);
    bracelet.clearAnimation(4);
    bracelet.animate(cfa, 1);
    ColorFlowAnimation cfa2 =
        new ColorFlowAnimation(0, 255, 0, 255, 0.1, 15, Direction.Backward, 7);
    bracelet.animate(cfa2, 2);
    ColorFlowAnimation cfa3 = new ColorFlowAnimation(0, 0, 255, 255, 0.1, 7, Direction.Forward, 0);
    bracelet.animate(cfa3, 3);
    ColorFlowAnimation cfa4 = new ColorFlowAnimation(0, 255, 0, 255, 0.1, 7, Direction.Backward, 0);
    bracelet.animate(cfa4, 4);
  }

  public void animate() {
    RainbowAnimation rb = new RainbowAnimation();
    bracelet.clearAnimation(1);
    bracelet.clearAnimation(2);
    bracelet.clearAnimation(3);
    bracelet.clearAnimation(4);
    rb.setBrightness(255);
    rb.setLedOffset(0);
    rb.setNumLed(300);
    rb.setSpeed(255);
    bracelet.animate(rb, 1);
    bracelet.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 255);
    bracelet.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_4_ControlTelem, 255);
    bracelet.setStatusFramePeriod(
        CANdleStatusFrame.CANdleStatusFrame_Status_5_PixelPulseTrain, 255);
    bracelet.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_6_BottomPixels, 255);
    bracelet.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_7_TopPixels, 255);
  }

  public void red() {
    setMulti(Color.kRed);
  }

  public void setMulti(Color color) {
    bracelet.clearAnimation(1);
    bracelet.clearAnimation(2);
    bracelet.clearAnimation(3);
    bracelet.clearAnimation(4);
    bracelet.setLEDs(
        ((int) (color.red * 255)), ((int) (color.green * 255)), ((int) (color.blue * 255)));
  }

  public Command setColor(Color color) {
    return this.run(
            () -> {
              bracelet.clearAnimation(1);
              bracelet.clearAnimation(2);
              bracelet.clearAnimation(3);
              bracelet.clearAnimation(4);
              bracelet.setLEDs(
                  ((int) (color.red * 255)),
                  ((int) (color.green * 255)),
                  ((int) (color.blue * 255)));
            })
        .withName("ColorSet");
  }

  public void off() {
    bracelet.clearAnimation(1);
    bracelet.clearAnimation(2);
    bracelet.clearAnimation(3);
    bracelet.clearAnimation(4);
    bracelet.setLEDs(0, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

/*   Can use this to add feedback light to buttons
 *   new Trigger(m_driveController::getStartButton).onTrue(new InstantCommand(() -> m_chassis.resetPose(new Pose2d())));
 *   new Trigger(m_driveController::getBButton).whileTrue(Commands.startEnd(m_feedback::red, m_feedback::off, m_feedback));
 *   new Trigger(m_driveController::getXButton).whileTrue(Commands.startEnd(()->m_feedback.multi(new Color(186,7,162)), m_feedback::off, m_feedback));
 *   new Trigger(m_driveController::getYButton).whileTrue(Commands.startEnd(()->m_feedback.multi(Color.fromHSV(316,47,10)), m_feedback::off, m_feedback));
 *   new Trigger(m_driveController::getAButton).whileTrue(Commands.startEnd(()->m_feedback.multi(new Color(0,255,0)), m_feedback::off, m_
 */
