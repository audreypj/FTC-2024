package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class ForceIntakeModeCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private IntakeSubsystem.Modes mode;

    public ForceIntakeModeCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.Modes mode) {
        this.intakeSubsystem = intakeSubsystem;
        this.mode = mode;

        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setMode(mode);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setMode(IntakeSubsystem.Modes.OFF);
    }
}
