package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.ArmRateCommand;
import org.firstinspires.ftc.teamcode.commands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.ForceIntakeModeCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commands.ZeroDrivebaseCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class RobotContainer {

    public enum AutonomousSelection {
        TEST, PARK
    }

    private final GamepadEx brandon, danny;
    private final Gamepad gamepad1, gamepad2;

    private final DrivebaseSubsystem drivebaseSubsystem;
    private final ArmSubsystem armSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;


    public RobotContainer(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        drivebaseSubsystem  = new DrivebaseSubsystem(hardwareMap);
        armSubsystem = new ArmSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);

        CommandScheduler.getInstance().registerSubsystem(drivebaseSubsystem);
        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
        CommandScheduler.getInstance().registerSubsystem(shooterSubsystem);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        brandon = new GamepadEx(this.gamepad1);
        danny = new GamepadEx(this.gamepad2);

        CommandScheduler.getInstance().setDefaultCommand(
                drivebaseSubsystem,
                new DefaultDriveCommand(
                        drivebaseSubsystem,
                        () -> {return Util.modifyJoystick(brandon.getLeftY(), 0.07);},
                        () -> {return Util.modifyJoystick(brandon.getLeftX(), 0.07);},
                        () -> {return -Util.modifyJoystick(brandon.getRightX(), 0.07);}));

            CommandScheduler.getInstance().setDefaultCommand(
                    armSubsystem,
                    new ArmRateCommand(
                            armSubsystem,
                            () -> {return Util.modifyJoystick(danny.getLeftY(), 0.07);},
                            () -> {return Util.modifyJoystick(danny.getRightY(), 0.07);}));

            configureButtonBindings();
    }

    private void configureButtonBindings() {
        new ButtonObject(brandon, GamepadKeys.Button.BACK)
                .whenActive(new ZeroDrivebaseCommand(drivebaseSubsystem));

        new ButtonObject(danny, GamepadKeys.Button.Y)
                .whenActive(new ArmCommand(armSubsystem, Constants.Arm.Setpoints.MAXIMUM_ANGLE));

        new ButtonObject(danny, GamepadKeys.Button.A)
                .whenActive(new ArmCommand(armSubsystem, Constants.Arm.Setpoints.STOWED));

        new ButtonObject(danny, GamepadKeys.Button.B)
                .whenActive(new ArmCommand(armSubsystem, Constants.Arm.ArmStates.SCORE));

        new Trigger(() -> danny.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3)
                .whileActiveContinuous(new ForceIntakeModeCommand(intakeSubsystem, IntakeSubsystem.Modes.INTAKE));

        new Trigger(() -> danny.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3)
                .whileActiveContinuous(new ForceIntakeModeCommand(intakeSubsystem, IntakeSubsystem.Modes.OUTTAKE));

        new ButtonObject(danny, GamepadKeys.Button.DPAD_UP)
                .whenActive(new ShooterCommand(shooterSubsystem, ShooterSubsystem.Modes.LAUNCH));
    }

    public Command getAutonomousCommand(AutonomousSelection autonomousSelection) {
        switch(autonomousSelection) {
            case TEST:
                return null;
            case PARK:
            default:
                return null;
        }
    }
}