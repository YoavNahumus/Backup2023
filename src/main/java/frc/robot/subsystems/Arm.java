package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

/**
 * The arm subsystem
 */
public class Arm extends SubsystemBase {
    private final TalonFX arm;
    private final DigitalInput armLimitSwitch = new DigitalInput(ArmConstants.ARM_LIMIT_SWITCH_ID);

    private final ArmFeedforward armFeedForward = new ArmFeedforward(ArmConstants.ARM_KS, ArmConstants.ARM_KG,
            ArmConstants.ARM_KV);

    /**
     * Creates a new Arm
     */
    public Arm() {
        arm = new TalonFX(ArmConstants.ARM_MOTOR_ID);
        setDefaultCommand(getCalibrateCommand());

        configDevices();
    }

    /**
     * Configures the devices to their default values
     */
    private void configDevices() {
        arm.configFactoryDefault();

        arm.setNeutralMode(NeutralMode.Brake);
        arm.config_kP(0, ArmConstants.ARM_KP);
        arm.config_kI(0, ArmConstants.ARM_KI);
        arm.config_kD(0, ArmConstants.ARM_KD);
        arm.config_kF(0, ArmConstants.ARM_KF);

        arm.configMotionCruiseVelocity(
                Math.toDegrees(ArmConstants.ARM_MAX_SPEED) * ArmConstants.PULSE_PER_DEGREE / 10);
        arm.configMotionAcceleration(
                Math.toDegrees(ArmConstants.ARM_MAX_ACCELERATION) * ArmConstants.PULSE_PER_DEGREE / 10);
    }

    /**
     * Sets the arm angle
     * 
     * @param angle The angle to set the arm to, in degrees
     */
    public void setArmAngle(double angle) {
        arm.set(ControlMode.MotionMagic, angle * ArmConstants.PULSE_PER_DEGREE, DemandType.ArbitraryFeedForward,
                armFeedForward.calculate(Math.toRadians(getArmAngle()), 0));
    }

    /**
     * Gets the arm angle
     * 
     * @return The arm angle, in degrees
     */
    public double getArmAngle() {
        return arm.getSelectedSensorPosition() / ArmConstants.PULSE_PER_DEGREE;
    }

    /**
     * Gets the arm limit switch state
     * 
     * @return True if the limit switch is closed, false otherwise
     */
    public boolean isArmLimitSwitchClosed() {
        return armLimitSwitch.get();
    }

    /**
     * Stops the arm
     */
    public void stop() {
        arm.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Sets the arm to the low position
     */
    public void setLow() {
        arm.set(ControlMode.PercentOutput, ArmConstants.CALIBRATE_POWER);
    }

    /**
     * Gets the set angle command
     * 
     * @param angle The angle to set the arm to, in degrees
     * @return The set angle command
     */
    public Command getSetAngleCommand(double angle) {
        Command command = new FunctionalCommand(() -> {}, () -> setArmAngle(angle), (interrupted) -> {
            if (interrupted)
                stop();
        }, () -> Math.abs(angle - getArmAngle()) < ArmConstants.ARM_ANGLE_TOLERANCE, this);
        command.setName("Set Arm Angle");
        return command;
    }

    /**
     * Gets the calibrate command
     * 
     * @return The calibrate command
     */
    public Command getCalibrateCommand() {
        Command command = new RunCommand(() -> {
            if (!isArmLimitSwitchClosed()) {
                setLow();
            } else {
                stop();
            }
        }, this).finallyDo((interrupted) -> stop());
        command.setName("Calibrate Arm");
        return command;
    }

    @Override
    public void periodic() {
        if (isArmLimitSwitchClosed())
            arm.setSelectedSensorPosition(ArmConstants.ARM_MIN_ANGLE * ArmConstants.PULSE_PER_DEGREE);
        if (isArmLimitSwitchClosed() && arm.getSelectedSensorVelocity() < 0) {
            stop();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Arm Limit Switch", this::isArmLimitSwitchClosed, null);
        builder.addDoubleProperty("Arm Angle", this::getArmAngle, null);
    }
}
