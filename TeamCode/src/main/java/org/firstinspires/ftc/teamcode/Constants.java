package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public final class Constants {

    public static final class Config {
        //enable when testing, just havent figured out implementing ftc dashboard yet
        public static final boolean SHOW_DEBUG_DATA = true;
    }

    public static final class Arm {
        //FIXME placeholders
        public static final double ARM_GEAR_RATIO = 0;
        public static final double ARM_SMALL_SPROCKET_CIRCUMFERENCE = 0;
        public static final double ARM_LARGE_SPROCKET_CIRCUMFERENCE = 0;

        public static final double ARM_ANGLE_OFFSET = 0;
        public static final double GRAVITY_PERCENT = 0.08;

        public static final class Setpoints {
            //FIXME very important fix pls
            public static final double PARALLEL = 0;
            public static final double BARE_MIN = -30;
            public static final double MAXIMUM_ANGLE = 50;
            public static final double STOWED = -10;
        }

        public static final class ArmStates {
            public static final ArmSubsystem.ArmState STOWED = new ArmSubsystem.ArmState(0, 0);
        }
    }

    public static final class Slide {
        public static final class Setpoints {
            public static final double STOWED = 0;
            public static final double MAXIMUM_EXTENSION = 20;
        }
    }

    public static final class Intake {
        public static final class ModePowers {
            //FIXME placeholder values
            public static final double OFF = 0;
            public static final double INTAKE = 0.3;

            public static final double OUTTAKE = -0.3;
        }
    }

    public static final class Shooter {
        public static final class Powers {
            public static final double OFF = 0;
            public static final double LAUNCH = 0.5;
            public static final double REVERSE = -0.5;
        }

        public static final class Timings {
            public static final double LAUNCH_DURATION = 0.5;
            public static final double REVERSE_DURATION = 0.5;
        }
    }
}
