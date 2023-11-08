package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Util;

public class ElevatorSubsystem extends SubsystemBase {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet;

    private final MotorEx elevatorMotor;
    private final SimpleServo wristMotor;
    private final PIDController elevatorController;

    private double elevatorOutput = 0;
    private double targetExtension = 0;
    private double targetAngle = 0;

    public ElevatorSubsystem(HardwareMap hMap) {

        //FIXME check if the motor has to be inverted
        elevatorMotor = new MotorEx(hMap, "eleMotor", Motor.GoBILDA.RPM_312);
        wristMotor = new SimpleServo(hMap, "wristMotor", 0, 360, AngleUnit.DEGREES);

        //The elevator always has to be inited in the lowest position
        elevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.resetEncoder();

        wristMotor.setPosition(0);

        //FIXME tune pid
        elevatorController = new PIDController(0.05, 0, 0);

        elevatorController.setTolerance(0.1);

        //set up ftc dashboard so i can test how the motor encoder/position works
    }

    private void setZero() {
        elevatorMotor.resetEncoder();
        wristMotor.setPosition(0);
    }

    private double getCurrentWristAngle() {
        return wristMotor.getAngle();
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
            targetAngle = Constants.Wrist.Setpoints.STOWED;
        } else if((getCurrentElevatorExtension() < Constants.Elevator.Setpoints.STOW_WRIST_EXTENSION || targetExtension < Constants.Elevator.Setpoints.STOW_WRIST_EXTENSION)) {
            targetAngle = Constants.Wrist.Setpoints.STOWED;
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

    public boolean atTargetElevator() {
        return Util.atTargetTolerance(getCurrentElevatorExtension(), 0.25);
    }

    public boolean atTargetAngle() {
        return Util.atTargetTolerance(getCurrentWristAngle(), 2);
    }

    public boolean atTargetAll() {
        return Util.atTargetTolerance(getCurrentElevatorExtension(), 0.25) && Util.atTargetTolerance(getCurrentWristAngle(), 2);
    }

    private void drivePeriodic() {
       elevatorOutput = elevatorController.calculate(getCurrentElevatorExtension(), determineTargetExtension());

       elevatorMotor.set(MathUtils.clamp(elevatorOutput + Constants.Elevator.GRAVITY_OFFSET_PERCENT, -0.5, 0.5));

       wristMotor.turnToAngle(determineTargetAngle());
    }

    @Override
    public void periodic() {

        drivePeriodic();

        if(Constants.Config.SHOW_DEBUG_DATA) {
            packet.put("targetExtension", targetExtension);
            packet.put("targetAngle", targetAngle);
            packet.put("currentExtension", getCurrentElevatorExtension());
            packet.put("currentAngle", getCurrentWristAngle());
            packet.put("extensionOutput", elevatorOutput);

            dashboard.sendTelemetryPacket(packet);
        }

    }
}
