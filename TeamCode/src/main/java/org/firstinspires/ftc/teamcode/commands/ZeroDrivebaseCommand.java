package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

public class ZeroDrivebaseCommand extends InstantCommand {

    private final DrivebaseSubsystem drivebaseSubsystem;

    public ZeroDrivebaseCommand(DrivebaseSubsystem drivebaseSubsystem) {
        this.drivebaseSubsystem = drivebaseSubsystem;

        addRequirements(this.drivebaseSubsystem);
    }

    @Override
    public void initialize() {
        drivebaseSubsystem.zeroGyroscope();
    }
}
