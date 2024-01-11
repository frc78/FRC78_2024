// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Systems.Chassis.Chassis;

/** TODO this is a stub for the driving command */
public class Drive extends Command{
    private Chassis chassis;
    private XboxController controller;

    public Drive(Chassis chassis, XboxController controller) {
        this.chassis = chassis;
        this.controller = controller;
    }

    @Override
    public void execute () {
        //drive
    }
}