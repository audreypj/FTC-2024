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
    private PIDController elevatorController;

    private double elevatorOutput = 0;
    private double targetExtension = 0;

    public ElevatorSubsystem(HardwareMap hMap) {

        //FIXME check if the motor has to be inverted
        elevatorMotor = new MotorEx(hMap, "eleMotor", Motor.GoBILDA.RPM_312);

        //The elevator always has to be inited in the lowest position
        elevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.resetEncoder();

        //FIXME tune pid
        elevatorController = new PIDController(0.05, 0, 0);

        //set up ftc dashboard so i can test how the motor encoder/position works
    }

    private void setZero() {
        elevatorMotor.resetEncoder();
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

    private double getCurrentElevatorExtension() {
        return ticksToInchesExtension(elevatorMotor.getCurrentPosition());
    }

    private double determineTargetExtension() {
        if(!currentOrTargetIsSafe()) {
            targetExtension = Constants.Elevator.Setpoints.MIN_EXTENSION_INCHES;
        }
        return targetExtension;
    }

    private boolean currentOrTargetIsSafe() {
        return withinSafeRange(getCurrentElevatorExtension())
               || withinSafeRange(targetExtension);
    }

    private boolean withinSafeRange(double position) {
        return position > Constants.Elevator.Setpoints.MIN_EXTENSION_INCHES
               || position < Constants.Elevator.Setpoints.MAX_EXTENSION_INCHES;
    }

    public boolean atTarget() {
        return false;
    }

    private void drivePeriodic() {
       elevatorOutput = elevatorController.calculate(getCurrentElevatorExtension(), determineTargetExtension());

       elevatorMotor.set(MathUtils.clamp(elevatorOutput, -0.3, 0.3));
    }

    @Override
    public void periodic() {

        drivePeriodic();

    }
}
