// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.ChassisUtils.Zone;
import frc.robot.utils.TrajectoryGenerator;

public class LeaveCommunity extends CommandBase {
    private final Chassis chassis;
    private Command command;

    public static enum TopOrBottom {
        TOP, BOTTOM;
    }


    private SendableChooser<TopOrBottom> chooser;

    /** Creates a new LeaveCommunity. */
    public LeaveCommunity(Chassis chassis) {
        this.chassis = chassis;
        chooser = new SendableChooser<>();
        chooser.setDefaultOption("Top", TopOrBottom.TOP);
        chooser.addOption("Bottom", TopOrBottom.BOTTOM);
        SmartDashboard.putData(chooser);
        addRequirements(chassis);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        command = new InstantCommand();
        TrajectoryGenerator generator = new TrajectoryGenerator(Alliance.Blue);

        Zone zone = Zone.fromRobotLocation(chassis.getPose().getTranslation());
        if (chooser.getSelected() == TopOrBottom.TOP) {
            switch (zone) {
                case COMMUNITY_MIDDLE:
                case COMMUNITY_BOTTOM:
                    generator.add(new Pose2d(new Translation2d(2.35, 4.89), Rotation2d.fromDegrees(180)),
                            Rotation2d.fromDegrees(0));
                case COMMUNITY_TOP:
                    generator.add(new Pose2d(new Translation2d(5.6, 4.89), Rotation2d.fromDegrees(180)),
                            Rotation2d.fromDegrees(0));
                default:
                    break;
            }
        } else {
            switch (zone) {
                case COMMUNITY_MIDDLE:
                case COMMUNITY_TOP:
                    generator.add(new Pose2d(new Translation2d(2.5, 0.5), Rotation2d.fromDegrees(180)),
                            Rotation2d.fromDegrees(0));
                case COMMUNITY_BOTTOM:
                    generator.add(new Pose2d(new Translation2d(5.6, 0.5), Rotation2d.fromDegrees(180)),
                            Rotation2d.fromDegrees(0));
                default:
                    break;
            }
        }
        generator.add(new Pose2d(new Translation2d(6.6, 2.75), Rotation2d.fromDegrees(235)), Rotation2d.fromDegrees(180));
        command = chassis.createPathFollowingCommand(false, generator.generate(chassis.getPose()));
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
        chassis.stop();
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
    
}
