package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class ArmRateCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private DoubleSupplier angleRate, extensionRate;

    public ArmRateCommand(ArmSubsystem armSubsystem, DoubleSupplier angleRate, DoubleSupplier extensionRate) {
        this.armSubsystem = armSubsystem;
        this.angleRate = angleRate;
        this.extensionRate = extensionRate;

        addRequirements(this.armSubsystem);
    }

    @Override
    public void execute() {
//        armSubsystem.setSlideMode(ArmSubsystem.SlideModes.POSITION);
//        armSubsystem.setTargetAngle(armSubsystem.getAngleDegrees() + angleRate.getAsDouble() * 2);
        armSubsystem.setSlideMode(ArmSubsystem.SlideModes.PERCENT);
        armSubsystem.setPercentControl(angleRate);
      //  armSubsystem.setTargetExtension(armSubsystem.getCurrentExtension() + extensionRate.getAsDouble());
        armSubsystem.setElevatorJoystick(extensionRate);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
