package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Util;

import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemBase {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final TelemetryPacket packet = new TelemetryPacket();

    private final MotorEx elevatorMotor;
    private final CRServo wristMotor;
    private final PIDController elevatorController;

    private double elevatorOutput = 0;
    private double targetExtension = 0;

    private DoubleSupplier wristControlJoystick;

    public ElevatorSubsystem(HardwareMap hMap) {

        //FIXME check if the motor has to be inverted
        elevatorMotor = new MotorEx(hMap, "eleMotor", Motor.GoBILDA.RPM_312);
        wristMotor = new CRServo(hMap, "wristMotor");

        elevatorMotor.setRunMode(Motor.RunMode.RawPower);

        //The elevator always has to be inited in the lowest position
        elevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.resetEncoder();

        //FIXME tune pid
        elevatorController = new PIDController(0.25, 0, 0);

        elevatorController.setTolerance(0.1);

        wristControlJoystick = () -> 0;

        //set up ftc dashboard so i can test how the motor encoder/position works
    }

    public void setWristControl(DoubleSupplier x) {
        wristControlJoystick = x;
    }

    private void setZero() {
        elevatorMotor.resetEncoder();
    }

    private double getCurrentWristAngle() {
        return 0;
    }

    //check if works
    private double ticksToInchesExtension(double ticks) {
        return (((38.2 / 25.4) * Math.PI) * ticks) / Motor.GoBILDA.RPM_312.getCPR();
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


    public double getCurrentElevatorExtension() {
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



    public boolean atTargetElevator() {
        return Util.atTargetTolerance(getCurrentElevatorExtension(), targetExtension, 0.5);
    }

    public boolean atTargetAll() {
        return atTargetElevator();
    }

    private void drivePeriodic() {
       elevatorOutput = MathUtils.clamp(elevatorController.calculate(getCurrentElevatorExtension(), determineTargetExtension()), -0.8, 0.8);
       //wristOutput = wristController.calculate(getCurrentWristAngle(), determineTargetAngle());

       elevatorMotor.set(elevatorOutput + Constants.Elevator.GRAVITY_OFFSET_PERCENT);
       //wristMotor.rotateByAngle(wristOutput);
        wristMotor.set(wristControlJoystick.getAsDouble());
    }

    @Override
    public void periodic() {

        drivePeriodic();

        if(Constants.Config.SHOW_DEBUG_DATA) {
            packet.put("targetExtension", targetExtension);
            packet.put("currentExtension", getCurrentElevatorExtension());
            packet.put("currentAngle", getCurrentWristAngle());
            packet.put("extensionOutput", elevatorOutput);

            dashboard.sendTelemetryPacket(packet);
        }

    }
}
