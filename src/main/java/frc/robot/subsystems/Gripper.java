package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

/**
 * The gripper subsystem
 */
public class Gripper extends SubsystemBase {
    private final TalonFX leftIntake, rightIntake;
    private final SimpleMotorFeedforward intakeFeedForward = new SimpleMotorFeedforward(GripperConstants.INTAKE_KS,
            GripperConstants.INTAKE_KV);

    /**
     * Creates a new Gripper
     */
    public Gripper() {
        leftIntake = new TalonFX(GripperConstants.LEFT_MOTOR_ID);
        rightIntake = new TalonFX(GripperConstants.RIGHT_MOTOR_ID);

        configDevices();
    }

    /**
     * Configures the devices to their default values
     */
    private void configDevices() {
        leftIntake.configFactoryDefault();
        rightIntake.configFactoryDefault();

        rightIntake.setInverted(true);
        leftIntake.setInverted(false);

        leftIntake.config_kP(0, GripperConstants.LEFT_KP);
        leftIntake.config_kI(0, GripperConstants.LEFT_KI);
        leftIntake.config_kD(0, GripperConstants.LEFT_KD);
        rightIntake.config_kP(0, GripperConstants.RIGHT_KP);
        rightIntake.config_kI(0, GripperConstants.RIGHT_KI);
        rightIntake.config_kD(0, GripperConstants.RIGHT_KD);

        rightIntake.follow(leftIntake);
    }

    /**
     * Sets the arm speed
     * 
     * @param speed The speed to set the arm to, in meters per second
     */
    public void setIntakeSpeed(double speed) {
        leftIntake.set(ControlMode.Velocity, speed * GripperConstants.PULSE_PER_METER / 10,
                DemandType.ArbitraryFeedForward,
                intakeFeedForward.calculate(speed));
    }

    /**
     * Gets the intake speed
     * 
     * @return The intake speed, in meters per second
     */
    public double getIntakeSpeed() {
        return leftIntake.getSelectedSensorVelocity() * 10 / GripperConstants.PULSE_PER_METER;
    }

    /**
     * Stops the intake
     */
    public void stop() {
        leftIntake.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Gets the set speed command
     * 
     * @param speed The speed to set the arm to, in meters per second
     * @return      The set speed command
     */
    public Command getSetSpeedCommand(double speed) {
        Command command = new StartEndCommand(() -> setIntakeSpeed(speed), this::stop, this);
        command.setName("Set Gripper Speed");
        return command;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Intake Speed", this::getIntakeSpeed, null);
    }
}
