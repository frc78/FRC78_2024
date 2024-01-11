// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import org.littletonrobotics.junction.LogFileUtil;
// import org.littletonrobotics.junction.LoggedRobot;
// import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.networktables.NT4Publisher;
// import org.littletonrobotics.junction.wpilog.WPILOGReader;
// import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    //#region AdvantageKit init
    // Logger logger = Logger.getInstance();

    // /* TODO COMMENT OUT FROM HERE */
    // logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    // logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    // logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    // logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    // logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    // switch (BuildConstants.DIRTY) {
    //   case 0:
    //     logger.recordMetadata("GitDirty", "All changes committed");
    //     break;
    //   case 1:
    //     logger.recordMetadata("GitDirty", "Uncomitted changes");
    //     break;
    //   default:
    //     logger.recordMetadata("GitDirty", "Unknown");
    //     break;
    // }
    // /* TO HERE */

    // if (isReal()) {
    //   logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
    //   logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    // } else {
    //   setUseTiming(false); // Run as fast as possible
    //   String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    //   logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    //   logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    // }

    // // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // // Logger.getInstance().disableDeterministicTimestamps()

    // // Start AdvantageKit logger
    // logger.start();
    // //#endregion
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
