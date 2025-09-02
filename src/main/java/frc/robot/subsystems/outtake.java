package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Ports;
import frc.robot.Ports.OuttakePorts;
import frc.robot.Constants;

public class Outtake extends SubsystemBase {

    private final CANSparkMax outtakeMotor;
    private final DigitalInput frontBeamBreak;
    private final DigitalInput middleBeamBreak;

    public Outtake() {
        // Initialize motor and sensors
        outtakeMotor = new CANSparkMax(Ports.OuttakePorts.OUTTAKE_MOTOR, MotorType.kBrushless);
        frontBeamBreak = new DigitalInput(Ports.OuttakePorts.FRONT_BB);
        middleBeamBreak = new DigitalInput(Ports.OuttakePorts.MIDDLE_BB);

        // Restore defaults and set idle mode
        outtakeMotor.restoreFactoryDefaults();
        outtakeMotor.setIdleMode(IdleMode.kBrake);
    }

    // Motor commands
    public Command runForward() {
        return run(() -> outtakeMotor.set(Constants.OuttakeSpeeds.MOTOR_SPEED));
    }

    public Command runReverse() {
        return run(() -> outtakeMotor.set(-Constants.OuttakeSpeeds.MOTOR_SPEED));
    }

    public Command stopMotor() {
        return runOnce(() -> outtakeMotor.set(0));
    }

    public Command setVoltageCmd(double voltage) {
        return run(() -> outtakeMotor.setVoltage(voltage));
    }

    // Beam break queries
    public boolean frontBeamBreakBroken() {
        return !frontBeamBreak.get(); // assuming normally closed
    }

    public boolean middleBeamBreakBroken() {
        return !middleBeamBreak.get(); // assuming normally closed
    }

    public boolean isCoralInPosition() {
        return !middleBeamBreakBroken() && frontBeamBreakBroken();
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