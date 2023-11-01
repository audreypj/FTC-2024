package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class SetIntakeModeCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;

    private IntakeSubsystem.RunModes runMode;

    public SetIntakeModeCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.RunModes runMode) {

        this.intakeSubsystem = intakeSubsystem;
        this.runMode = runMode;

        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setRunMode(runMode);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setRunMode(IntakeSubsystem.RunModes.OFF);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
