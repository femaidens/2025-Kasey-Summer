package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Ports;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final CANSparkMax leaderMotor;
    private final CANSparkMax followerMotor;
    private final RelativeEncoder relativeEncoder;
    private final DigitalInput bottomLimitSwitch;

    private final PIDController pid;
    private final ElevatorFeedforward ff;

    public Elevator() {
        // Motors
        leaderMotor = new CANSparkMax(Ports.ElevatorPorts.ELEVATOR_LEADER_MOTOR, MotorType.kBrushless);
        followerMotor = new CANSparkMax(Ports.ElevatorPorts.ELEVATOR_FOLLOWER_MOTOR, MotorType.kBrushless);

        // Restore factory defaults
        leaderMotor.restoreFactoryDefaults();
        followerMotor.restoreFactoryDefaults();

        // Idle mode
        leaderMotor.setIdleMode(IdleMode.kBrake);
        followerMotor.setIdleMode(IdleMode.kBrake);

        // Follower follows leader
        followerMotor.follow(leaderMotor, true);

        // Encoder
        relativeEncoder = leaderMotor.getEncoder();

        // Limit switch
        bottomLimitSwitch = new DigitalInput(Ports.ElevatorPorts.ELEVATOR_BOTTOM_LIMIT_SWITCH_PORT);

        // PID and Feedforward
        pid = new PIDController(
            Constants.ElevatorPIDValues.kP,
            Constants.ElevatorPIDValues.kI,
            Constants.ElevatorPIDValues.kD
        );
        pid.setTolerance(0.1);

        ff = new ElevatorFeedforward(
            Constants.ElevatorFeedforwardConstants.kS,
            Constants.ElevatorFeedforwardConstants.kG,
            Constants.ElevatorFeedforwardConstants.kV,
            Constants.ElevatorFeedforwardConstants.kA
        );
    }

    // PID voltage control
    private void elevatorPID(double current, double setpoint) {
        leaderMotor.setVoltage(pid.calculate(current, setpoint));
    }

    // Commands
    public Command runElevatorMotorCmd() {
        return run(() -> leaderMotor.set(Constants.ElevatorSpeeds.MOTOR_SPEED));
    }

    public Command reverseMotor() {
        return run(() -> leaderMotor.set(-Constants.ElevatorSpeeds.MOTOR_SPEED));
    }

    public Command stopElevatorMotorCmd() {
        return runOnce(() -> leaderMotor.set(0));
    }

    public Command moveToPosition(double setpoint) {
        return run(() -> elevatorPID(relativeEncoder.getPosition(), setpoint));
    }

    public Command setVoltageCmd(double voltage) {
        return run(() -> leaderMotor.setVoltage(voltage));
    }

    public Command resetEncoder() {
        return runOnce(() -> relativeEncoder.setPosition(0));
    }

    public Command setLevel(double setpoint) {
        return run(() -> elevatorPID(relativeEncoder.getPosition(), setpoint));
    }

    // Getter
    public double getHeight() {
        return relativeEncoder.getPosition();
    }

    public boolean hitBottomLimit() {
        return !bottomLimitSwitch.get(); // normally closed
    }

    @Override
    public void periodic() {
        // Called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // Called once per scheduler run during simulation
    }
}
}