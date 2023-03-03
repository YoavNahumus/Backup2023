package frc.robot;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utils.Rectangle;

public class Constants {
    public static final double JOYSTICK_DEADBAND = 0.1; // the deadband for the joysticks
    public static final double JOYSTICK_ANGLE_DEADBAND = 0.2; // the deadband for the angle of the joysticks
    public static final double JOYSTICK_IDLE_DEADBAND = 0.3; // the deadband to check if the joystick is idle

    public static final Rectangle RAMP = new Rectangle(2.91, 1.51, 4.85, 3.98); // in meters, blue alliance
    public static final Rectangle OPEN_AREA = new Rectangle(4.85, 0.0, 11.69, 8.02); // in meters, blue alliance
    public static final Rectangle ENTRANCE_BOTTOM = new Rectangle(2.91, 0.0, 4.85, 1.51); // in meters, blue alliance
    public static final Rectangle ENTRANCE_TOP = new Rectangle(2.91, 3.98, 4.85, 5.49); // in meters, blue alliance
    public static final Rectangle COMMUNITY_BOTTOM = new Rectangle(0.0, 0.0, 2.91, 1.51); // in meters, blue alliance
    public static final Rectangle COMMUNITY_TOP = new Rectangle(0.0, 3.98, 2.91, 5.49); // in meters, blue alliance
    public static final Rectangle COMMUNITY_MIDDLE = new Rectangle(0, 1.51, 2.91, 3.98); // in meters, blue alliance
    public static final Rectangle LOADING_ZONE = new Rectangle(11.69, 5.55, 16.54, 8.02); // in meters, blue alliance

    public static final double FIELD_WIDTH = 16.54; // in meters
    public static final double FIELD_HEIGHT = 8.02; // in meters
    public static final double AUTONOMOUS_CLIMB_SPEED = 1;
    public static final int CYCLES_PER_SECOND = 50;

    /**
     * The Vision constants.
     */
    public static final class VisionConstants {
        public static final NetworkTable LIMELIGHT_TABLE1 = NetworkTableInstance.getDefault().getTable("limelight");
        public static final NetworkTable LIMELIGHT_TABLE2 = NetworkTableInstance.getDefault().getTable("limelight-iii");

        public static final double MAX_DISTANCE_FOR_LIMELIGHT = 4.5;
    }

    /**
     * The Swerve Modules constants.
     */
    public static class SwerveModuleConstants {
        public final double angleOffset;
        public final int moveMotorID;
        public final int angleMotorID;
        public final int absoluteEncoderID;

        public static final double VELOCITY_KP = 2e-5;
        public static final double VELOCITY_KI = 6e-5;
        public static final double VELOCITY_KS = 0.0128;
        public static final double VELOCITY_KV = 0.227675;
        public static final SimpleMotorFeedforward VELOCITY_FF = new SimpleMotorFeedforward(VELOCITY_KS, VELOCITY_KV);
        public static final double ANGLE_KP = 0.2;
        public static final double ANGLE_KI = 0.002;
        public static final double ANGLE_KD = 5;
        public static final double MAX_ACCUM_INTEGRAL = 80000;

        public static final double PPR_FALCON = 2048;
        public static final double WHEEL_PERIMITER = 0.1016 * Math.PI; // meters
        public static final double GEAR_RATIO_VEL = 8.14;
        public static final double PULSE_PER_METER = PPR_FALCON * GEAR_RATIO_VEL / WHEEL_PERIMITER;

        public static final double GEAR_RATIO_ANGLE = 12.8;
        public static final double PULSE_PER_DEGREE = PPR_FALCON * GEAR_RATIO_ANGLE / 360;

        /**
         * Creates a new SwerveModuleConstants.
         * 
         * @param angleOffset       The offset of the absolute encoder from the module
         * @param moveMotorID       The CAN ID of the drive motor
         * @param angleMotorID      The CAN ID of the angle motor
         * @param absoluteEncoderID The CAN ID of the absolute encoder
         */
        private SwerveModuleConstants(double angleOffset, int moveMotorID, int angleMotorID, int absoluteEncoderID) {
            this.angleOffset = angleOffset;
            this.moveMotorID = moveMotorID;
            this.angleMotorID = angleMotorID;
            this.absoluteEncoderID = absoluteEncoderID;
        }

        public static final SwerveModuleConstants FRONT_LEFT = new SwerveModuleConstants(30.5859375, 7, 8, 11);
        public static final SwerveModuleConstants FRONT_RIGHT = new SwerveModuleConstants(302.6953125, 5, 6, 13);
        public static final SwerveModuleConstants BACK_LEFT = new SwerveModuleConstants(226.93359375, 1, 2, 10);
        public static final SwerveModuleConstants BACK_RIGHT = new SwerveModuleConstants(108.544921875, 3, 4, 12);
    }

    /**
     * The Swerve Drive constants.
     */
    public static class ChassisConstants {
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(0.26515, 0.2215), // front left
                new Translation2d(0.26515, -0.2215), // front right
                new Translation2d(-0.26515, 0.2215), // back left
                new Translation2d(-0.26515, -0.2215) // back right
        );

        public static final int GYRO_ID = 14;

        public static final double MAX_SPEED = 4.2; // meters per second
        public static final double MAX_AUTO_ACCELERATION = 4; // meters per second squared
        public static final double MAX_AUTO_SPEED = 4;
        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(MAX_AUTO_SPEED, MAX_AUTO_ACCELERATION);
        public static final double MAX_DRIVE_SPEED = 4;
        public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

        public static final double AUTO_TRANSLATION_KP = 2;
        public static final double AUTO_TRANSLATION_KI = 0.3;
        public static final double AUTO_ROTATION_KP = 1;
        public static final double AUTO_ROTATION_KI = 0.3;

        public static final double TELEOP_ROTATION_KP = 4;
        public static final double TELEOP_ROTATION_KI = 0.3;

        public static final double TELEOP_ANGLE_TOLERANCE = Math.PI / 120;

        public static final double AUTO_ANGLE_TOLERANCE = Math.PI / 180;
        public static final double AUTO_TRANSLATION_TOLERANCE = 0.02;
    }

    /**
     * The Gripper constants.
     */
    public static class GripperConstants {
        public static final int LEFT_MOTOR_ID = -1;
        public static final int RIGHT_MOTOR_ID = -1;

        public static final double INTAKE_KS = -1;
        public static final double INTAKE_KV = -1;

        public static final double LEFT_KP = -1;
        public static final double LEFT_KI = -1;
        public static final double LEFT_KD = -1;

        public static final double RIGHT_KP = -1;
        public static final double RIGHT_KI = -1;
        public static final double RIGHT_KD = -1;

        public static final double PPR_FALCON = 2048;

        public static final double INTAKE_GEAR_RATIO = 1;
        public static final double INTAKE_WHEEL_PERIMITER = 0.05715 * Math.PI;
        public static final double PULSE_PER_METER = PPR_FALCON * INTAKE_GEAR_RATIO / INTAKE_WHEEL_PERIMITER;

        public static final double SHOOT_HIGH_SPEED = -1;
        public static final double SHOOT_MID_SPEED = -1;
        public static final double SHOOT_LOW_SPEED = -1;
        public static final double SHOOT_TIME = 0.5;

        public static final double INTAKE_SPEED = -1;
    }

    /**
     * The Arm constants.
     */
    public static class ArmConstants {
        public static final int ARM_MOTOR_ID = -1;
        public static final int ARM_LIMIT_SWITCH_ID = 0;

        public static final double ARM_KS = -1;
        public static final double ARM_KG = -1;
        public static final double ARM_KV = -1;

        public static final double ARM_KP = -1;
        public static final double ARM_KI = -1;
        public static final double ARM_KD = -1;
        public static final double ARM_KF = -1;

        public static final double ARM_MAX_SPEED = -1;
        public static final double ARM_MAX_ACCELERATION = -1;

        public static final double PPR_FALCON = 2048;

        public static final double ARM_GEAR_RATIO = 80;
        public static final double PULSE_PER_DEGREE = PPR_FALCON * ARM_GEAR_RATIO / 360;

        public static final double ARM_MIN_ANGLE = 0;
        public static final double CALIBRATE_POWER = -0.4;

        public static final double ARM_SHOOT_ANGLE = 45;

        public static final double ARM_ANGLE_TOLERANCE = 0.5;
    }
}
