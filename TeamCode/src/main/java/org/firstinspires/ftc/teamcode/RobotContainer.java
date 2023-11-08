package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.LinearSlideCommand;
import org.firstinspires.ftc.teamcode.commands.ForceIntakeModeCommand;
import org.firstinspires.ftc.teamcode.commands.LinearSlideRateCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommand;
import org.firstinspires.ftc.teamcode.commands.WristRateCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.Optional;

public class RobotContainer {

    private GamepadEx brandon, danny;
    private Gamepad gamepad1, gamepad2;

    private final DrivebaseSubsystem drivebaseSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public RobotContainer(HardwareMap hardwareMap, Optional<Gamepad> gamepad1, Optional<Gamepad> gamepad2) {
        drivebaseSubsystem  = new DrivebaseSubsystem(hardwareMap);
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        CommandScheduler.getInstance().registerSubsystem(drivebaseSubsystem);
        CommandScheduler.getInstance().registerSubsystem(elevatorSubsystem);
        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);

        if(gamepad1.isPresent()) {
            this.gamepad1 = gamepad1.get();
            this.gamepad2 = gamepad2.get();
            brandon = new GamepadEx(this.gamepad1);
            danny = new GamepadEx(this.gamepad2);

            CommandScheduler.getInstance().setDefaultCommand(
                    drivebaseSubsystem,
                    new DefaultDriveCommand(
                            drivebaseSubsystem,
                            brandon::getLeftY,
                            brandon::getLeftX,
                            brandon::getRightX));

            CommandScheduler.getInstance().setDefaultCommand(
                    elevatorSubsystem,
                    new LinearSlideRateCommand(
                            elevatorSubsystem,
                            danny::getLeftY));

            configureButtonBindings();
        }
    }

    public RobotContainer(HardwareMap hardwareMap) {
        this(hardwareMap, Optional.empty(), Optional.empty());
    }

    private void configureButtonBindings() {

        new ButtonObject(danny, GamepadKeys.Button.Y)
                .whenActive(new LinearSlideCommand(elevatorSubsystem, Constants.Elevator.Setpoints.HIGH_SETPOINT));

        new ButtonObject(danny, GamepadKeys.Button.B)
                .whenActive(new LinearSlideCommand(elevatorSubsystem, Constants.Elevator.Setpoints.MID_HEIGHT_INCHES));

        new ButtonObject(danny, GamepadKeys.Button.A)
                .whenActive(new LinearSlideCommand(elevatorSubsystem, Constants.Elevator.Setpoints.MIN_EXTENSION_INCHES));

        new ButtonObject(danny, GamepadKeys.Button.LEFT_BUMPER)
                .whenActive(new WristCommand(elevatorSubsystem, Constants.Wrist.Setpoints.STOWED));

        new ButtonObject(danny, GamepadKeys.Button.RIGHT_BUMPER)
                .whenActive(new WristCommand(elevatorSubsystem, Constants.Wrist.Setpoints.SCORE));

        new Trigger(() -> danny.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3)
                .whenActive(new ForceIntakeModeCommand(intakeSubsystem, IntakeSubsystem.RunModes.INTAKE));

        new Trigger(() -> danny.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3)
                .whenActive(new ForceIntakeModeCommand(intakeSubsystem, IntakeSubsystem.RunModes.OUTTAKE));

    }

}
