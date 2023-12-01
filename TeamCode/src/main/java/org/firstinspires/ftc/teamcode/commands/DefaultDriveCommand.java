package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {

    private final DrivebaseSubsystem drivebaseSubsystem;

    private DoubleSupplier translationXSupplier, translationYSupplier, rightX;

    public DefaultDriveCommand(DrivebaseSubsystem drivebaseSubsystem, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rightX) {
        this.drivebaseSubsystem = drivebaseSubsystem;

        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rightX = rightX;

        addRequirements(drivebaseSubsystem);
    }

    @Override
    public void execute() {
//        drivebaseSubsystem.drive(
//                ChassisSpeeds.fromFieldRelativeSpeeds(
//                        translationXSupplier.getAsDouble(),
//                        translationYSupplier.getAsDouble(),
//                        rightX.getAsDouble(),
//                        drivebaseSubsystem.getConsistentGyroAngle()));
        drivebaseSubsystem.driveFieldCentricJoystick(translationXSupplier, translationYSupplier, translationXSupplier);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivebaseSubsystem.driveFieldCentricJoystick(
                () -> 0,
                () -> 0,
                () -> 0);
    }

}
