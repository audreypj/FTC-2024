package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.Optional;

public class ArmSubsystem extends SubsystemBase {

    private HardwareMap hMap;

    private MotorEx armMotor, armMotorTwo;

    private PIDController armController;

    public class ArmState {
        public double angle, extension;
        public ArmState(Optional<Double> angle, Optional<Double> extension) {
            this.angle = angle.orElse((double) 0);
            this.extension = extension.orElse((double) 0); //FIXME should actually like make these valid default points
        }


    }

    public ArmSubsystem(HardwareMap hMap) {

        this.hMap = hMap;

        armMotor = new MotorEx(hMap, "armMotor", Motor.GoBILDA.RPM_60);
        armMotorTwo = new MotorEx(hMap, "armMotorTwo", Motor.GoBILDA.RPM_60);

        armMotor.resetEncoder();
        armMotorTwo.resetEncoder();

        armMotorTwo.setInverted(true);

        armController = new PIDController(0.003, 0.001, 0.001);
        armController.setTolerance(3);

    }

    private double tickToDegrees(double ticks) {
        //FIXME make sure math works
        return ((ticks / 2786) * Constants.Arm.ARM_GEAR_RATIO) * 360;
        //currentPos/total circumference = degrees/360
        //currentPos = (tickPos / cpr) * gear ratio * totalCircumferece
    }

    private double getRawAnglePosition() {
        return 0;
    }

    public double getCorrectedAnglePosition() {
        return 0;
    }

    @Override
    public void periodic() {

    }
}
