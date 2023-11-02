package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    private final MotorEx elevatorMotor;
    private final MotorEx wristMotor;
    private final PIDController elevatorController;
    private final PIDController wristController;

    private double elevatorOutput = 0;
    private double wristOutput = 0;
    private double targetExtension = 0;
    private double targetAngle = 0;

    public ElevatorSubsystem(HardwareMap hMap) {

        //FIXME check if the motor has to be inverted
        elevatorMotor = new MotorEx(hMap, "eleMotor", Motor.GoBILDA.RPM_312);
        wristMotor = new MotorEx(hMap, "wristMotor", Motor.GoBILDA.RPM_312);

        //The elevator always has to be inited in the lowest position
        elevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.resetEncoder();

        wristMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        wristMotor.resetEncoder();

        //FIXME tune pid
        elevatorController = new PIDController(0.05, 0, 0);
        wristController = new PIDController(0.05, 0, 0);

        elevatorController.setTolerance(0.1);
        wristController.setTolerance(1);

        //set up ftc dashboard so i can test how the motor encoder/position works
    }

    private void setZero() {
        elevatorMotor.resetEncoder();
    }

    private double ticksToDegrees(double ticks) {
        return (ticks * 360) / wristMotor.getCPR();
    }

    private double getCurrentWristAngle() {
        return ticksToDegrees(wristMotor.getCurrentPosition());
    }

    //check if works
    private double ticksToInchesExtension(double ticks) {
        return ((38.2 * Math.PI) * ticks) / Motor.GoBILDA.RPM_312.getCPR();
    }

    private double extensionToHeightInches(double inches) {
        return (Math.sin(Constants.Elevator.ELEVATOR_SLANT_ANGLE) * inches);
    }

    public void setElevatorTargetInches(double targetPosition) {
        this.targetExtension =
                MathUtils.clamp(
                        targetPosition,
                        Constants.Elevator.Setpoints.MIN_EXTENSION_INCHES,
                        Constants.Elevator.Setpoints.MAX_EXTENSION_INCHES);
    }

    //FIXME proper range
    public void setWristTargetAngle(double angle) {
        this.targetAngle =
                MathUtils.clamp(
                        angle,
                        0,
                        180);
    }

    public double getWristTargetAngle() {
        return targetAngle;
    }

    private double determineTargetAngle() {
        if(!currentOrTargetAngleIsSafe()) {
            targetAngle = 0;
        }
        return targetAngle;
    }

    private double getCurrentElevatorExtension() {
        return ticksToInchesExtension(elevatorMotor.getCurrentPosition());
    }

    private double determineTargetExtension() {
        if(!currentOrTargetExtensionIsSafe()) {
            targetExtension = Constants.Elevator.Setpoints.MIN_EXTENSION_INCHES;
        }
        return targetExtension;
    }

    private boolean currentOrTargetExtensionIsSafe() {
        return withinSafeExtensionRange(getCurrentElevatorExtension())
               || withinSafeExtensionRange(targetExtension);
    }

    private boolean withinSafeExtensionRange(double position) {
        return position > Constants.Elevator.Setpoints.MIN_EXTENSION_INCHES
               || position < Constants.Elevator.Setpoints.MAX_EXTENSION_INCHES;
    }

    //FIXME actual safe range
    private boolean withinSafeAngleRange(double angle) {
        return angle > 0
                || angle < 180;
    }

    private boolean currentOrTargetAngleIsSafe() {
        return withinSafeAngleRange(getCurrentWristAngle())
                || withinSafeAngleRange(targetAngle);
    }

    public boolean atTarget() {
        return false;
    }

    private void drivePeriodic() {
       elevatorOutput = elevatorController.calculate(getCurrentElevatorExtension(), determineTargetExtension());
       wristOutput = wristController.calculate(getCurrentWristAngle(), determineTargetAngle());

       elevatorMotor.set(MathUtils.clamp(elevatorOutput, -0.3, 0.3));
    }

    @Override
    public void periodic() {

        drivePeriodic();

    }
}
