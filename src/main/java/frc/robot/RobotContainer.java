// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GripperConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.GoUpRamp;
import frc.robot.commands.GotoCommunity;
import frc.robot.commands.GotoNodes;
import frc.robot.commands.LeaveCommunity;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Gripper;
import frc.robot.utils.Utils;

public class RobotContainer {
    private static RobotContainer instance;
    private final CommandXboxController main = new CommandXboxController(0);
    private final CommandXboxController secondary = new CommandXboxController(1);
    private final Chassis chassis;
    private final Gripper gripper;
    private final Arm arm;

    private final Command auto;

    private RobotContainer() {
        chassis = new Chassis();
        gripper = new Gripper();
        arm = new Arm();
        SmartDashboard.putData(chassis);
        SmartDashboard.putData(gripper);
        SmartDashboard.putData(arm);

        chassis.setDefaultCommand(new Drive(chassis, main.getHID()));

        SmartDashboard.putData(CommandScheduler.getInstance());

        auto = new GotoNodes(chassis, secondary, () -> arm.getSetAngleCommand(ArmConstants.ARM_SHOOT_ANGLE)).andThen(
                gripper.getSetSpeedCommand(GripperConstants.SHOOT_HIGH_SPEED).withTimeout(0.5),
                new LeaveCommunity(chassis), new GoUpRamp(chassis, -2.5));
        configureBindings();
    }

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    private void configureBindings() {
        Command unload = new GotoCommunity(chassis)
                .andThen(new GotoNodes(chassis, secondary, () -> arm.getSetAngleCommand(ArmConstants.ARM_SHOOT_ANGLE)),
                        gripper.getSetSpeedCommand(GripperConstants.SHOOT_HIGH_SPEED).withTimeout(0.5));
        unload = unload.until(() -> Utils.hasInput(main.getHID()));

        main.x().onTrue(unload);

        main.a().whileTrue(gripper.getSetSpeedCommand(GripperConstants.INTAKE_SPEED));

        main.b().whileTrue(arm.getSetAngleCommand(ArmConstants.ARM_SHOOT_ANGLE)
                .andThen(gripper.getSetSpeedCommand(GripperConstants.SHOOT_HIGH_SPEED)));

        main.y().whileTrue(arm.getSetAngleCommand(ArmConstants.ARM_SHOOT_ANGLE)
                .andThen(gripper.getSetSpeedCommand(GripperConstants.SHOOT_HIGH_SPEED)));

        main.rightBumper().whileTrue(gripper.getSetSpeedCommand(GripperConstants.SHOOT_LOW_SPEED));
    }

    public Command getAutonomousCommand() {
        return auto;
    }
}
