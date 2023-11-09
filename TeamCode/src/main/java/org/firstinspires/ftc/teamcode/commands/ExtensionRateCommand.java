package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class ExtensionRateCommand extends CommandBase {

    private ArmSubsystem armSubsystem;
    private DoubleSupplier rate;

    public ExtensionRateCommand(ArmSubsystem armSubsystem, DoubleSupplier rate) {
        this.armSubsystem = armSubsystem;
        this.rate = rate;

        addRequirements(this.armSubsystem);
    }

    @Override
    public void execute() {
         armSubsystem.setTargetExtension(armSubsystem.getCurrentExtension() + rate.getAsDouble());
    }

}
