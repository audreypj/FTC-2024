package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Util;

import java.util.Optional;
import java.util.function.DoubleSupplier;

public class ArmSubsystem extends SubsystemBase {

    public enum SlideModes {
        POSITION, PERCENT
    }

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final TelemetryPacket packet = new TelemetryPacket();

    private final MotorEx armMotor, armMotorTwo, extensionMotor;

    private final PIDController armController, extensionController;

    private double targetAngle;
    private double targetExtension;

    private SlideModes currentSlideMode;
    private DoubleSupplier percentControl;

    public static class ArmState {
        public double angle, extension;
        public ArmState(Optional<Double> angle, Optional<Double> extension) {
            this.angle = angle.orElse((double) Constants.Arm.Setpoints.STOWED);
            this.extension = extension.orElse((double) Constants.Slide.Setpoints.STOWED); //FIXME should actually like make these valid default points
        }

        public ArmState(double angle, double extension) {
            this(Optional.of(angle), Optional.of(extension));
        }

        public ArmState(double angle) {
            this(Optional.of(angle), Optional.empty());
        }
    }

    public ArmSubsystem(HardwareMap hMap) {
        armMotor = new MotorEx(hMap, "armMotor", Motor.GoBILDA.RPM_60);
        armMotorTwo = new MotorEx(hMap, "armMotorTwo", Motor.GoBILDA.RPM_60);
        extensionMotor = new MotorEx(hMap, "slideMotor", Motor.GoBILDA.RPM_312); //FIXME check correct rpm

        armMotor.setRunMode(Motor.RunMode.RawPower);
        armMotorTwo.setRunMode(Motor.RunMode.RawPower);
        extensionMotor.setRunMode(Motor.RunMode.RawPower);

        armMotor.resetEncoder();
        armMotorTwo.resetEncoder();
        extensionMotor.resetEncoder();

        armMotorTwo.setInverted(true);

        armController = new PIDController(0.003, 0.001, 0.001);
        extensionController = new PIDController(0.003, 0, 0.001);
        armController.setTolerance(3);
        extensionController.setTolerance(0.1);

        targetAngle = 0;
        targetExtension = 0;

        currentSlideMode = SlideModes.POSITION;
        percentControl = () -> 0;
    }

    private double ticksToInchesExtension(double ticks) {
        return (((38.2 / 25.4) * Math.PI) * ticks) / extensionMotor.getCPR();
    }

    private double tickToDegrees(double ticks) {
        //FIXME make sure math works
        return ((ticks / 2786) * Constants.Arm.ARM_GEAR_RATIO) * 360;
        //currentPos/total circumference = degrees/360
        //currentPos = (tickPos / cpr) * gear ratio * totalCircumferece
    }

    //make parallel to ground 0 degrees using offset
    public double getAngleDegrees() {
        return tickToDegrees(armMotor.getCurrentPosition());
    }

    public double getCurrentExtension() {
        return -ticksToInchesExtension(extensionMotor.getCurrentPosition());
    }

    public void setSlideMode(SlideModes mode) {
        currentSlideMode = mode;
    }

    //0 degrees at parallel
    public void setTargetAngle(double angle) {
        targetAngle = MathUtils.clamp(angle, Constants.Arm.Setpoints.STOWED, Constants.Arm.Setpoints.MAXIMUM_ANGLE);
    }

    public void setPercentControl(DoubleSupplier percent) {
        percentControl = percent;
    }

    public void setTargetExtension(double extension) {
        targetExtension = MathUtils.clamp(extension, Constants.Slide.Setpoints.STOWED, Constants.Slide.Setpoints.MAXIMUM_EXTENSION);
    }

    public void setArmState(ArmState armState) {
        setTargetAngle(armState.angle);
        setTargetExtension(armState.extension);
    }

    private double calculateGravityOffset() {
        return Math.sin(getAngleDegrees() + Constants.Arm.STOWED_OFFSET) * Constants.Arm.GRAVITY_PERCENT;
    }

    private boolean isAngleSafe(double angle) {
        return angle > Constants.Arm.Setpoints.STOWED
                && angle < Constants.Arm.Setpoints.MAXIMUM_ANGLE;
    }

    private boolean isExtensionSafe(double extension) {
        return extension > Constants.Slide.Setpoints.STOWED
                && extension < Constants.Slide.Setpoints.MAXIMUM_EXTENSION;
    }

    private boolean currentOrTargetAngleSafe() {
        return isAngleSafe(getAngleDegrees())
                && isAngleSafe(targetAngle);
    }

    private boolean currentOrTargetExtensionSafe() {
        return isExtensionSafe(getCurrentExtension())
                && isExtensionSafe(targetExtension);
    }

    private double determineTargetAngle() {
        if(!currentOrTargetAngleSafe()) {
            targetAngle = Constants.Arm.Setpoints.STOWED;
        }
        return targetAngle;
    }

    private double determineTargetExtension() {
        if(!currentOrTargetExtensionSafe()) {
            targetExtension = Constants.Slide.Setpoints.STOWED;
        } else if(getAngleDegrees() < Constants.Arm.Setpoints.STOWED
                || getAngleDegrees() > Constants.Arm.Setpoints.MAXIMUM_ANGLE
                || targetAngle < Constants.Arm.Setpoints.STOWED
                || targetAngle > Constants.Arm.Setpoints.MAXIMUM_ANGLE) {
            targetExtension = Constants.Slide.Setpoints.STOWED;
        }
        return targetExtension;
    }

    public boolean atTargetExtension() {
        return Util.atTargetTolerance(getCurrentExtension(), targetExtension, 0.25);
    }

    public boolean atTargetAngle() {
        return Util.atTargetTolerance(getAngleDegrees(), targetAngle, 0.25);
    }

    public boolean atTargetAll() {
        return atTargetExtension() && atTargetAngle();
    }

    private void posPeriodic() {
        double armPower = armController.calculate(getAngleDegrees(), determineTargetAngle()) + calculateGravityOffset();

        double extensionOutput = extensionController.calculate(getCurrentExtension(), determineTargetExtension());

        armMotor.set(armPower);
        armMotorTwo.set(armPower);
        extensionMotor.set(extensionOutput);
    }

    private void percentPeriodic() {

        double armPower = percentControl.getAsDouble() + calculateGravityOffset();
        double extensionOutput = extensionController.calculate(getCurrentExtension(), determineTargetExtension());

        armMotor.set(armPower);
        armMotorTwo.set(armPower);
        extensionMotor.set(extensionOutput);
    }

    @Override
    public void periodic() {
        applyMode();

        if(Constants.Config.SHOW_DEBUG_DATA) {
            packet.put("targetAngle", targetAngle);
            packet.put("targetExtension", targetExtension);
            packet.put("angleDegrees", getAngleDegrees());
            packet.put("currentExtension", getCurrentExtension());

            dashboard.sendTelemetryPacket(packet);
        }
    }

    private void applyMode() {
        switch(currentSlideMode) {
            case POSITION:
                posPeriodic();
                break;
            case PERCENT:
                percentPeriodic();
                break;
        }
    }
}