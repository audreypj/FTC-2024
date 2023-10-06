package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    private HardwareMap hMap;

    private Motor elevatorMotor;
    private PIDController elevatorController;

    private double elevatorOutput = 0;
    private double targetHeight = 0;

    public ElevatorSubsystem(HardwareMap hMap) {

        this.hMap = hMap;

        elevatorMotor = new MotorEx(hMap, "eleMotor");

        elevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        elevatorController = new PIDController(0.01, 0, 0);

        //set up ftc dashboard so i can test how the motor encoder/position works
    }

    private void setZero() {
        elevatorMotor.resetEncoder();
    }

    private double ticksToInches(double ticks) {
        //FIXME actually do the math
        return 0;
    }

    public void setElevatorTargetInches(double targetPosition) {
        this.targetHeight =
                MathUtils.clamp(
                        targetPosition,
                        Constants.Arm.Setpoints.MIN_HEIGHT,
                        Constants.Arm.Setpoints.MAX_HEIGHT);
    }

    public double getCurrentElevatorHeight() {
        return ticksToInches(elevatorMotor.getCurrentPosition());
    }

    //if the slide extends past the min or max extension then pizdecs - have to fix it
    private boolean withinSafeRange(double position) {
        return position;
    }

    private void drivePeriodic() {

       elevatorOutput =
               elevatorController.calculate(
                       getCurrentElevatorHeight(), targetHeight);

       elevatorMotor.set(elevatorOutput);
    }

    @Override
    public void periodic() {

    }
}
