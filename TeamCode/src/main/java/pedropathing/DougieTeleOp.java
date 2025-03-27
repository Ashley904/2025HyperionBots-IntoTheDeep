package pedropathing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "Official TeleOp")
public class DougieTeleOp extends LinearOpMode {

    DougieArmSubSystem armSubSystem;

    DcMotor front_left_motor;
    DcMotor front_right_motor;
    DcMotor back_left_motor;
    DcMotor back_right_motor;


    /**  Bicubic dynamic acceleration + deceleration variables ***/
    private static final double minimumDriveSpeed = 0.3;
    public static double cubicTerm = 0.5;
    public static double linearTerm = 0.4;


    /**  PID Variables ***/
    public static double headingKp = 3;
    public static double headingKi = 0;
    public static double headingKd = 0.275;


    /**  Field Centric Stuff ***/
    IMU imu;

    double targetHeading;
    double headingCorrection;

    PIDController FieldCentricPIDController;


    /**  Booleans ***/
    private boolean lastDpadUpState;
    private boolean lastDpadDownState;
    private boolean lockHeading = true;
    private boolean lastLeftBumperState = false;

    ElapsedTime releaseTimer;


    /** Drive Mode Toggling **/
    private String currentDriveMode = "Robot Centric";

    public void runOpMode(){
        armSubSystem = new DougieArmSubSystem(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        FieldCentricPIDController = new PIDController(headingKp,headingKi,headingKd);

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

        armSubSystem.VerticalArmIdlePosition();
        CommandScheduler.getInstance().run();

        releaseTimer = new ElapsedTime();

        telemetry.addData("Status: ", "Ready to start");
        telemetry.update();

        waitForStart();

        releaseTimer.reset();
        releaseTimer.startTime();

        while(opModeIsActive()){
            DriveModeToggling();
            ArmPositionToggling();
            BackgroundOpModeTasks();

        }
    }

    private void DriveModeToggling() {

        // Toggling robot centric drive
        if (gamepad1.dpad_up && !lastDpadUpState && !currentDriveMode.equals("Robot Centric")) {
            lastDpadUpState = true;
            currentDriveMode = "Robot Centric";
            Gamepad.RumbleEffect driveModeRumbleUpdate = new Gamepad.RumbleEffect.Builder()
                    .addStep(1.0, 1.0, 200)
                    .build();
            gamepad1.runRumbleEffect(driveModeRumbleUpdate);
        } else if (!gamepad1.dpad_up) lastDpadUpState = false;


        // Toggling field centric drive
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
    private void FieldCentricDrive(){

        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        if (Math.abs(gamepad1.right_stick_x) <= 0.01) {
            if (!lockHeading) targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            lockHeading = true;
        } else lockHeading = false;


        if (lockHeading) {
            double headingError = targetHeading - currentHeading;
            headingError = Math.IEEEremainder(headingError, 2 * Math.PI);

            if (Math.abs(headingError) < Math.toRadians(2)) {
                headingCorrection = 0;
            } else {
                headingCorrection = FieldCentricPIDController.calculate(headingError);
            }
        }

        double adjustedDrivingSpeed = cubicTerm * Math.pow(gamepad1.right_trigger, 3) + linearTerm * gamepad1.right_trigger; // Bicubic dynamic speed control
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

        double rotX = x * Math.cos(-currentHeading) - y * Math.sin(-currentHeading);
        double rotY = x * Math.sin(-currentHeading) + y * Math.cos(-currentHeading);

        if (lockHeading) rx = headingCorrection;

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
    private void RobotCentricDrive(){
        double adjustedDrivingSpeed = cubicTerm * Math.pow(gamepad1.right_trigger, 3) + linearTerm * gamepad1.right_trigger; // Bicubic dynamic speed control
        adjustedDrivingSpeed = 1.0 - adjustedDrivingSpeed;

        adjustedDrivingSpeed = Math.max(adjustedDrivingSpeed, minimumDriveSpeed);

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


    private void BackgroundOpModeTasks(){

        FieldCentricPIDController.setPID(headingKp, headingKi, headingKd);

        armSubSystem.VerticalPIDFSlideControl();
        armSubSystem.HorizontalPIDFSlideControl();

        CommandScheduler.getInstance().run();
    }


    private void ArmPositionToggling(){
        // Position for specimen collection
        if (gamepad1.left_bumper) {
            armSubSystem.PositionForSpecimenCollection();
            lastLeftBumperState = true;
            releaseTimer.reset();
        }
        // Position for specimen hanging
        else if (lastLeftBumperState) {
            if (releaseTimer.milliseconds() >= 200) {
                armSubSystem.PositionForSpecimenScoring();
                lastLeftBumperState = false;
            }
        }


        // Hang specimen
        if (gamepad1.a) {
            armSubSystem.ScoreSpecimen();
        }
    }


}
