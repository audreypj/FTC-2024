package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

public class DrivebaseCommand extends CommandBase {

    private DrivebaseSubsystem drivebaseSubsystem;

    private DoubleSupplier leftY, leftX, rightX;

    public DrivebaseCommand(DrivebaseSubsystem drivebaseSubsystem, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX) {
        this.drivebaseSubsystem = drivebaseSubsystem;

        this.leftY = leftY;
        this.leftX = leftX;
        this.rightX = rightX;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        drivebaseSubsystem.driveMotors(leftY, leftX, rightX);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
