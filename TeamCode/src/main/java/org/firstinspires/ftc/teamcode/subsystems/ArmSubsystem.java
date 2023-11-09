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

    private double targetAngle;

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

        targetAngle = 0;

    }

    private double tickToDegrees(double ticks) {
        //FIXME make sure math works
        return ((ticks / 2786) * Constants.Arm.ARM_GEAR_RATIO) * 360;
        //currentPos/total circumference = degrees/360
        //currentPos = (tickPos / cpr) * gear ratio * totalCircumferece
    }


    private double getRawAnglePosition() {
        return tickToDegrees(armMotor.getCurrentPosition());
    }

    //make parallel to ground 0 degrees using offset
    public double getCorrectedAnglePosition() {
        return getRawAnglePosition() + Constants.Arm.ARM_ANGLE_OFFSET;
    }

    private double calculateGravityOffset() {
        return Math.sin(getCorrectedAnglePosition() - 90) * Constants.Arm.GRAVITY_PERCENT;
    }

    private boolean isAngleSafe(double angle) {
        return angle > Constants.Arm.Setpoints.BARE_MIN
                && angle < Constants.Arm.Setpoints.MAXIMUM_ANGLE;
    }

    private boolean currentOrTargetAngleSafe() {
        return isAngleSafe(getCorrectedAnglePosition())
                && isAngleSafe(targetAngle);
    }

    private double determineTargetAngle() {

        if(!currentOrTargetAngleSafe()) {
            targetAngle = Constants.Arm.Setpoints.STOWED;
        }

        return targetAngle;
    }

    private void drivePeriodic() {
        double armPower = armController.calculate(getCorrectedAnglePosition(), determineTargetAngle()) + calculateGravityOffset();

        armMotor.set(armPower);
        armMotorTwo.set(armPower);
    }

    @Override
    public void periodic() {

    }
}
