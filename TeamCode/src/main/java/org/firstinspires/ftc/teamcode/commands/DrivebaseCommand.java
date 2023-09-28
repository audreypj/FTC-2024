package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

import java.util.function.DoubleSupplier;

public class DrivebaseCommand extends CommandBase {

    private DrivebaseSubsystem drivebaseSubsystem;

    private DoubleSupplier leftY, leftX, rightX;

    public DrivebaseCommand(DrivebaseSubsystem drivebaseSubsystem, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX) {
        this.drivebaseSubsystem = drivebaseSubsystem;

        this.leftY = leftY;
        this.leftX = leftX;
        this.rightX = rightX;

        drivebaseSubsystem.driveMotors(leftY, leftX, rightX);

        addRequirements(drivebaseSubsystem);
    }

    @Override
    public void execute() {
        //drivebaseSubsystem.driveMotors(leftY, leftX, rightX);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivebaseSubsystem.driveMotors(
                () -> {return 0;},
                () -> {return 0;},
                () -> {return 0;});
    }
}
