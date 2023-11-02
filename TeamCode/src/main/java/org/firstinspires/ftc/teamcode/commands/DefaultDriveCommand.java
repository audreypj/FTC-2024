package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {

    private DrivebaseSubsystem drivebaseSubsystem;

    private DoubleSupplier leftY, leftX, rightX;

    public DefaultDriveCommand(DrivebaseSubsystem drivebaseSubsystem, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX) {
        this.drivebaseSubsystem = drivebaseSubsystem;

        this.leftY = leftY;
        this.leftX = leftX;
        this.rightX = rightX;
    }

    @Override
    public void execute() {
        drivebaseSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        leftY.getAsDouble(),
                        leftX.getAsDouble(),
                        rightX.getAsDouble(),
                        drivebaseSubsystem.getConsistentGyroAngle()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
