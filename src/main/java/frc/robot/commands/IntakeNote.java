// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {

  Intake intaker;
  double setSpeedT;
  double setSpeedB;

  /** Creates a new IntakeNote. */
  public IntakeNote(Intake intake1, double setSpeedT, double setSpeedB) {
    // Use addRequirements() here to declare subsystem dependencies.
    intaker = intake1;
    this.setSpeedT = setSpeedT;
    this.setSpeedB = setSpeedB;
    addRequirements(intaker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intaker.intakeCTRL(setSpeedT, setSpeedB);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intaker.intakeSTOP();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
