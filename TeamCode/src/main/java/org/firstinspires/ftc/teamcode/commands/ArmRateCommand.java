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
        armSubsystem.setTargetAngle(armSubsystem.getAngleDegrees() + angleRate.getAsDouble());
        armSubsystem.setTargetExtension(armSubsystem.getCurrentExtension() + extensionRate.getAsDouble());
    }
}
