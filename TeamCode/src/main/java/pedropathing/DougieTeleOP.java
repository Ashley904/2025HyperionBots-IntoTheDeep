package pedropathing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "Official TeleOp")
public class DougieTeleOP extends LinearOpMode {

    DougieArmSubSystem armSubsystem;

    private DcMotor front_left_motor;
    private DcMotor front_right_motor;
    private DcMotor back_left_motor;
    private DcMotor back_right_motor;

    // ---------- Drive Mode Toggling Variables ----------
    String currentDriveMode = "Robot Centric";
    boolean lastDpadUpState = false;
    boolean lastDpadDownState = false;
    IMU imu;
    ElapsedTime timer = new ElapsedTime();

    // -----------Drive Settings----------
    double minimumDriveSpeed = 0.25;

    public static double a = 0.6; // Cubic term coefficient
    public static double b = 0.4; // Linear term coefficient

    // ----------Misc-----------
    private boolean hasRumbledEndgame = false;

    boolean lastLeftBumperState = false;
    boolean lastLeftTriggerStateGamePad2 = false;

    private final ElapsedTime releaseTimer = new ElapsedTime();

    // ----------Heading Lock----------
    public static double headingKp = 0.565;
    public static double headingKi = 0;
    public static double headingKd = 0.04215;

    double targetHeading;
    double headingCorrection;

    boolean headingLock = true;

    PIDController headingPIDController;

    // -----------Arm position toggling----------
    boolean positionedForSpecimenHanging = false;

    @Override
    public void runOpMode() {
        headingPIDController = new PIDController(headingKp, headingKi, headingKd);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        front_left_motor = hardwareMap.get(DcMotor.class, "FL");
        front_right_motor = hardwareMap.get(DcMotor.class, "FR");
        back_left_motor = hardwareMap.get(DcMotor.class, "BL");
        back_right_motor = hardwareMap.get(DcMotor.class, "BR");

        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left_motor.setDirection(DcMotor.Direction.REVERSE);
        front_right_motor.setDirection(DcMotor.Direction.FORWARD);
        back_left_motor.setDirection(DcMotor.Direction.REVERSE);
        back_right_motor.setDirection(DcMotor.Direction.FORWARD);

        // SubSystems
        armSubsystem = new DougieArmSubSystem(hardwareMap);
        telemetry.addData("Status:", "Ready To Starttt...");
        telemetry.update();

        waitForStart();

        armSubsystem.IdlePosition();

        if (isStopRequested()) return;
        timer.reset();
        timer.startTime();

        while (opModeIsActive()) {
            // Calling Functions
            ToggleDriveMode();
            OpModeBackgroundTasks();
            ArmPositionToggling();

            // Updates
            armSubsystem.UpdateVerticalSlidePIDFControl();
            armSubsystem.UpdateHorizontalSlidePIDFControl();

            CommandScheduler.getInstance().run();
        }
    }

    private void RobotCentricDrive() {
        double adjustedDrivingSpeed = a * Math.pow(gamepad1.right_trigger, 3) + b * gamepad1.right_trigger; // Bicubic dynamic speed control
        adjustedDrivingSpeed = 1.0 - adjustedDrivingSpeed;

        adjustedDrivingSpeed = Math.max(adjustedDrivingSpeed, minimumDriveSpeed); // Clamping min drive speed

        double y = -gamepad1.left_stick_y * adjustedDrivingSpeed;
        double x = gamepad1.left_stick_x * adjustedDrivingSpeed;
        double rx = gamepad1.right_stick_x * adjustedDrivingSpeed;

        double frontLeftPower = (y + x + rx);
        double backLeftPower = (y - x + rx);
        double frontRightPower = (y - x - rx);
        double backRightPower = (y + x - rx);

        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
        );

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }

        front_left_motor.setPower(frontLeftPower);
        back_left_motor.setPower(backLeftPower);
        front_right_motor.setPower(frontRightPower);
        back_right_motor.setPower(backRightPower);
    }

    private void FieldCentricDrive() {
        // Enabling and disabling heading lock
        if (Math.abs(gamepad1.right_stick_x) <= 0.05) {
            if (!headingLock) targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            headingLock = true;
        } else {
            headingLock = false;
        }
        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (headingLock) {
            double headingError = targetHeading - robotHeading;
            headingError = Math.IEEEremainder(headingError, 2 * Math.PI);

            if (Math.abs(headingError) < Math.toRadians(2)) {
                headingCorrection = 0;
            } else {
                headingCorrection = headingPIDController.calculate(headingError);
            }
        }

        double adjustedDrivingSpeed = a * Math.pow(gamepad1.right_trigger, 3) + b * gamepad1.right_trigger; // Bicubic dynamic speed control
        adjustedDrivingSpeed = 1.0 - adjustedDrivingSpeed;

        adjustedDrivingSpeed = Math.max(adjustedDrivingSpeed, minimumDriveSpeed);

        double y = -gamepad1.left_stick_y * adjustedDrivingSpeed;
        double x = gamepad1.left_stick_x * adjustedDrivingSpeed;
        double rx = gamepad1.right_stick_x * adjustedDrivingSpeed;

        // Reset imu yaw
        if (gamepad1.start) {
            imu.resetYaw();
            Gamepad.RumbleEffect headingResetRumble = new Gamepad.RumbleEffect.Builder()
                    .addStep(1.0, 1.0, 450)
                    .build();
            gamepad1.runRumbleEffect(headingResetRumble);

            targetHeading = 0;
        }

        double rotX = x * Math.cos(-robotHeading) - y * Math.sin(-robotHeading);
        double rotY = x * Math.sin(-robotHeading) + y * Math.cos(-robotHeading);

        if (headingLock) {
            rx = headingCorrection;
        }

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        front_left_motor.setPower(frontLeftPower);
        back_left_motor.setPower(backLeftPower);
        front_right_motor.setPower(frontRightPower);
        back_right_motor.setPower(backRightPower);
    }

    private void ToggleDriveMode() {
        //----------Toggling Robot Centric Drive----------
        if (gamepad1.dpad_up && !lastDpadUpState && !currentDriveMode.equals("Robot Centric")) {
            lastDpadUpState = true;
            currentDriveMode = "Robot Centric";
            Gamepad.RumbleEffect driveModeRumbleUpdate = new Gamepad.RumbleEffect.Builder()
                    .addStep(1.0, 1.0, 200)
                    .build();
            gamepad1.runRumbleEffect(driveModeRumbleUpdate);
        } else if (!gamepad1.dpad_up) lastDpadUpState = false;

        //----------Toggling Field Centric Drive----------
        if (gamepad1.dpad_down && !lastDpadDownState && !currentDriveMode.equals("Field Centric")) {
            lastDpadDownState = true;
            currentDriveMode = "Field Centric";
            Gamepad.RumbleEffect driveModeRumbleUpdate = new Gamepad.RumbleEffect.Builder()
                    .addStep(1.0, 1.0, 200)
                    .build();
            gamepad1.runRumbleEffect(driveModeRumbleUpdate);
        } else if (!gamepad1.dpad_down) lastDpadDownState = false;

        if (currentDriveMode.equals("Robot Centric")) RobotCentricDrive();
        else if (currentDriveMode.equals("Field Centric")) FieldCentricDrive();
    }

    private void OpModeBackgroundTasks() {
        headingPIDController.setPID(headingKp, headingKi, headingKd);

        // End Game Rumble
        if (timer.time() >= 90 && !hasRumbledEndgame) {
            Gamepad.RumbleEffect endGameRumble = new Gamepad.RumbleEffect.Builder()
                    .addStep(1.0, 1.0, 2000)
                    .build();
            gamepad1.runRumbleEffect(endGameRumble);
            gamepad2.runRumbleEffect(endGameRumble);

            hasRumbledEndgame = true;
        }
    }

    private void ArmPositionToggling() {


        // Position for specimen collection
        if (gamepad1.left_bumper) {
            armSubsystem.PositionForSpecimenCollection();
            lastLeftBumperState = true;
            releaseTimer.reset();
        }
        // Position for specimen hanging
        else if (lastLeftBumperState) {
            if (releaseTimer.milliseconds() >= 200) {
                positionedForSpecimenHanging = true;
                armSubsystem.PositionForSpecimenHanging();
                lastLeftBumperState = false;
            } else {
                positionedForSpecimenHanging = false;
            }
        }


        // Hang specimen
        if (gamepad1.a && positionedForSpecimenHanging) {
            armSubsystem.ScoreSpecimenOntoHighBar();
        }


        // Positioning for sample collection
        armSubsystem.ManualHorizontalSlideControl(gamepad2.right_stick_x); // Manual slide override

        if (gamepad2.left_trigger > 0.05) {
            double rotationServoInput = gamepad2.left_stick_x;
            armSubsystem.PositionForSampleCollection(rotationServoInput);

            lastLeftTriggerStateGamePad2 = true;
            releaseTimer.reset();
        } else if (lastLeftTriggerStateGamePad2) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> armSubsystem.CollectSample()),
                            new WaitCommand(885),
                            new InstantCommand(() -> armSubsystem.IdlePosition())
                    )
            );

            lastLeftTriggerStateGamePad2 = false;
            armSubsystem.isManualOverrideEnabled = false;
        }


        // Position the arm into the idle position
        if(gamepad2.a) armSubsystem.IdlePosition();
    }
}