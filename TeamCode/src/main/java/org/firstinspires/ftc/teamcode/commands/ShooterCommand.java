package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    private ShooterSubsystem.Modes mode;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, ShooterSubsystem.Modes mode) {
        this.shooterSubsystem = shooterSubsystem;
        this.mode = mode;

        addRequirements(this.shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setMode(mode);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.getMode() == ShooterSubsystem.Modes.OFF;
    }

}
