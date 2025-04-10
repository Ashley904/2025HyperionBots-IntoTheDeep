package pedropathing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "Official TeleOp with Vision")
public class DougieTeleOp extends LinearOpMode {

    DougieArmSubSystem armSubSystem;

    DcMotor front_left_motor;
    DcMotor front_right_motor;
    DcMotor back_left_motor;
    DcMotor back_right_motor;

    private Limelight3A limelight;
    private boolean visionModeActive = false;
    private boolean lastVisionToggleState = false;

    // Vision alignment parameters
    public static double IMAGE_CENTER_X = 320.0;
    public static double ALIGNMENT_OFFSET = -9.65;
    public static double MAX_STRAFE_POWER = 0.26;
    public static double MIN_EFFECTIVE_STRAFE_POWER = 0.2215;
    public static double FINAL_ALIGNMENT_TOLERANCE = 1;
    public static int REQUIRED_STABLE_FRAMES = 6;

    public static double kP = 0.0065;
    public static double kI = 0.000465;
    public static double kD = 0.0002;
    public static double kF = 0.0055;

    public static double headingKp = 3;
    public static double headingKi = 0;
    public static double headingKd = 0.275;

    public static double slideExtensionOffsetInches = 6.889764;
    public static double logCorrectionFactor = 18.05;
    public static double horizontalSlideTicksPerInch = 57.55;

    public static double ROTATION_OFFSET_DEGREES = 64.5;
    public static int SERVO_ROTATION_DELAY_MS = 650;

    private PIDController alignmentPID;
    private PIDController headingLockPID;

    private double targetHeading = 0;
    private double lastCorrectedWorldY = 0;
    private double smoothedAngle = 0;
    private double lockedAngle = 0;
    private double lockedSlideTarget = 0;
    private double lastRotationOffsetApplied = Double.NaN;

    private boolean alignmentFinished = false;
    private boolean slideTargetLocked = false;
    private boolean waitingForSlideRetraction = false;
    private boolean rotationServoPositionLocked = false;
    private boolean collectionTriggered = false;
    private boolean servoRotationStarted = false;
    private long servoRotationStartTime = 0;
    private int stableFrameCount = 0;
    private double lastStableX = 0;

    // Drive parameters
    private static final double minimumDriveSpeed = 0.3;
    public static double cubicTerm = 0.5;
    public static double linearTerm = 0.4;

    // Heading control parameters
    public static double fieldCentricHeadingKp = 3;
    public static double fieldCentricHeadingKi = 0;
    public static double fieldCentricHeadingKd = 0.275;
    public static double fieldCentricHeadingKf = 0.05;

    IMU imu;
    double fieldCentricTargetHeading;
    double headingCorrection;
    PIDController FieldCentricPIDController;

    // Button state tracking
    private boolean lastDpadUpState;
    private boolean lastDpadDownState;
    private boolean lastGamepad2DpadUpState;
    private boolean lastGamepad2DpadDownState;
    private boolean lockHeading = true;
    private boolean lastLeftBumperState = false;
    private boolean wasHoldingLeftTrigger = false;
    private boolean wasScoringWithLeftTrigger = false;

    ElapsedTime releaseTimer;
    private String currentDriveMode = "Robot Centric";
    private String currentSampleCollectionMode = "Specimen Mode";

    @Override
    public void runOpMode() {
        armSubSystem = new DougieArmSubSystem(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        FieldCentricPIDController = new PIDController(fieldCentricHeadingKp, fieldCentricHeadingKi, fieldCentricHeadingKd);
        headingLockPID = new PIDController(headingKp, headingKi, headingKd);

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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();

        alignmentPID = new PIDController(kP, kI, kD);

        releaseTimer = new ElapsedTime();
        armSubSystem.IntakeOpModeIdlePosition();
        armSubSystem.OuttakeOpModeIdlePosition();
        CommandScheduler.getInstance().run();
        armSubSystem.updateServos();

        telemetry.addData("Status: ", "Ready to start");
        telemetry.update();
        waitForStart();

        armSubSystem.IntakeOpModeIdlePosition();
        armSubSystem.OuttakeOpModeIdlePosition();
        CommandScheduler.getInstance().run();
        armSubSystem.updateServos();
        armSubSystem.HorizontalPIDFSlideControl();

        releaseTimer.reset();
        releaseTimer.startTime();

        targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        fieldCentricTargetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        while (opModeIsActive()) {
            // Toggle vision mode with Y button
            if (gamepad1.y && !lastVisionToggleState) {
                visionModeActive = !visionModeActive;
                if (visionModeActive) {
                    resetVisionSystem();
                    gamepad1.runRumbleEffect(new Gamepad.RumbleEffect.Builder().addStep(1.0, 1.0, 150).build());
                } else {
                    gamepad1.runRumbleEffect(new Gamepad.RumbleEffect.Builder().addStep(0.5, 0.5, 150).build());
                }
            }
            lastVisionToggleState = gamepad1.y;

            if (visionModeActive) {
                handleVisionAlignment();
            } else {
                DriveModeToggling();
                SampleModeToggling();
                ArmPositionToggling();
            }

            BackgroundOpModeTasks();
            updateTelemetry();
        }
    }

    private void resetVisionSystem() {
        alignmentFinished = false;
        slideTargetLocked = false;
        waitingForSlideRetraction = true;
        rotationServoPositionLocked = false;
        collectionTriggered = false;
        servoRotationStarted = false;
        stableFrameCount = 0;
        armSubSystem.horizontalSlideTargetPosition = 0;
        armSubSystem.LimeLightIntakeIdlePosition();
        armSubSystem.PositionForSampleScanning();
    }

    private void handleVisionAlignment() {
        handleArmControl();
        processVisionAndAlignment();
    }

    private void processVisionAndAlignment() {
        if (waitingForSlideRetraction) {
            telemetry.addLine("Waiting for slide to retract...");
            applyMotorPowers(0, getHeadingCorrection());
            telemetry.update();
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || result.getPythonOutput() == null || result.getPythonOutput().length < 7) {
            telemetry.addLine("No target detected");
            stableFrameCount = 0;
            applyMotorPowers(0, getHeadingCorrection());
            telemetry.update();
            return;
        }

        double[] output = result.getPythonOutput();
        boolean locked = output[0] == 1.0 || output[1] == 1.0 || output[2] == 1.0;
        if (!locked) {
            telemetry.addLine("No block locked");
            stableFrameCount = 0;
            applyMotorPowers(0, getHeadingCorrection());
            telemetry.update();
            return;
        }

        double angle = output[3];
        double centerX = output[4];
        double worldY = output[6];
        double targetX = IMAGE_CENTER_X + ALIGNMENT_OFFSET;
        double error = centerX - targetX;

        alignmentPID.setPID(kP, kI, kD);
        double pidOutput = -alignmentPID.calculate(centerX, targetX);
        double ff = Math.signum(error) * kF;
        double strafePower = pidOutput + ff;

        if (Math.abs(strafePower) < MIN_EFFECTIVE_STRAFE_POWER && Math.abs(error) > FINAL_ALIGNMENT_TOLERANCE) {
            strafePower = Math.copySign(MIN_EFFECTIVE_STRAFE_POWER, strafePower);
        }
        strafePower = Math.max(-MAX_STRAFE_POWER, Math.min(MAX_STRAFE_POWER, strafePower));

        // Check for stable alignment
        if (Math.abs(error) <= FINAL_ALIGNMENT_TOLERANCE) {
            if (stableFrameCount == 0 || Math.abs(centerX - lastStableX) < 2.0) {
                stableFrameCount++;
                lastStableX = centerX;
            } else {
                stableFrameCount = 0;
            }
        } else {
            stableFrameCount = 0;
        }

        if (!alignmentFinished && stableFrameCount >= REQUIRED_STABLE_FRAMES) {
            alignmentFinished = true;
        }

        smoothedAngle = 0.8 * smoothedAngle + 0.2 * angle;

        // Start servo rotation as soon as we have a stable angle, even before full alignment
        if (!servoRotationStarted && stableFrameCount >= 3) {
            lockedAngle = ((smoothedAngle % 360) + 360) % 360;
            if (lockedAngle > 180) lockedAngle = 360 - lockedAngle;

            double finalAngle = lockedAngle + ROTATION_OFFSET_DEGREES;
            finalAngle = Math.max(0.0, Math.min(180.0, finalAngle));
            double mappedServoPosition = finalAngle / 180.0;
            armSubSystem.horizontalRotationServo.setTargetPosition(mappedServoPosition);

            lastRotationOffsetApplied = ROTATION_OFFSET_DEGREES;
            servoRotationStarted = true;
            servoRotationStartTime = System.currentTimeMillis();
            rotationServoPositionLocked = true;
        }

        if (rotationServoPositionLocked && ROTATION_OFFSET_DEGREES != lastRotationOffsetApplied) {
            double finalAngle = lockedAngle + ROTATION_OFFSET_DEGREES;
            finalAngle = Math.max(0.0, Math.min(180.0, finalAngle));
            double mappedServoPosition = finalAngle / 180.0;
            armSubSystem.horizontalRotationServo.setTargetPosition(mappedServoPosition);

            lastRotationOffsetApplied = ROTATION_OFFSET_DEGREES;
        }

        if (alignmentFinished && rotationServoPositionLocked && !slideTargetLocked && worldY > 0) {
            double correctedWorldY = worldY + (0.015 * Math.pow(worldY, 2)) - 0.75;
            correctedWorldY = 0.8 * correctedWorldY + 0.2 * lastCorrectedWorldY;
            lastCorrectedWorldY = correctedWorldY;

            double logCorrection = logCorrectionFactor * Math.log10(correctedWorldY);
            double scaledY = correctedWorldY + slideExtensionOffsetInches - logCorrection;
            lockedSlideTarget = scaledY * horizontalSlideTicksPerInch;
            armSubSystem.horizontalSlideTargetPosition = lockedSlideTarget;

            slideTargetLocked = true;
        }

        // Only proceed with collection if:
        // 1. We're aligned
        // 2. Slide is at target
        // 3. Servo has had time to rotate
        if (alignmentFinished && rotationServoPositionLocked && slideTargetLocked && !collectionTriggered) {
            long timeSinceServoRotation = System.currentTimeMillis() - servoRotationStartTime;
            if (timeSinceServoRotation >= SERVO_ROTATION_DELAY_MS) {
                armSubSystem.LimelightPositionForSampleCollection();
                sleep(300);
                armSubSystem.LimelightCollectSample();
                collectionTriggered = true;
            }
        }

        applyMotorPowers(alignmentFinished ? 0 : strafePower, getHeadingCorrection());

        telemetry.addData("centerX", centerX);
        telemetry.addData("targetX", targetX);
        telemetry.addData("error", error);
        telemetry.addData("StrafePower", strafePower);
        telemetry.addData("HeadingCorrection", getHeadingCorrection());
        telemetry.addData("LockedSlideTarget", lockedSlideTarget);
        telemetry.addData("SmoothedAngle", smoothedAngle);
        telemetry.addData("LockedAngle", lockedAngle);
        telemetry.addData("OffsetAngle", ROTATION_OFFSET_DEGREES);
        telemetry.addData("FinalAngle", lockedAngle + ROTATION_OFFSET_DEGREES);
        telemetry.addData("ServoTargetPos", (lockedAngle + ROTATION_OFFSET_DEGREES) / 180.0);
        telemetry.addData("StableFrameCount", stableFrameCount);
        telemetry.addData("CollectionTriggered", collectionTriggered);
        telemetry.addData("TimeSinceServoRotation", System.currentTimeMillis() - servoRotationStartTime);
    }

    private void applyMotorPowers(double strafe, double headingCorrection) {
        double fl = strafe + headingCorrection;
        double fr = -strafe - headingCorrection;
        double bl = -strafe + headingCorrection;
        double br = strafe - headingCorrection;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        front_left_motor.setPower(fl / max);
        front_right_motor.setPower(fr / max);
        back_left_motor.setPower(bl / max);
        back_right_motor.setPower(br / max);
    }

    private double getHeadingCorrection() {
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double headingError = targetHeading - currentHeading;
        headingError = Math.IEEEremainder(headingError, 2 * Math.PI);
        if (Math.abs(headingError) < Math.toRadians(1.5)) return 0;
        return headingLockPID.calculate(headingError);
    }

    private void handleArmControl() {
        armSubSystem.VerticalPIDFSlideControl();
        armSubSystem.HorizontalPIDFSlideControl();
        armSubSystem.updateServos();
        CommandScheduler.getInstance().run();

        if (waitingForSlideRetraction && Math.abs(armSubSystem.currentHorizontalSlidePosition) <= 50) {
            waitingForSlideRetraction = false;
        }
    }

    private void DriveModeToggling() {
        if (gamepad1.dpad_up && !lastDpadUpState && !currentDriveMode.equals("Robot Centric")) {
            lastDpadUpState = true;
            currentDriveMode = "Robot Centric";
            gamepad1.runRumbleEffect(new Gamepad.RumbleEffect.Builder().addStep(1.0, 1.0, 200).build());
        } else if (!gamepad1.dpad_up) lastDpadUpState = false;

        if (gamepad1.dpad_down && !lastDpadDownState && !currentDriveMode.equals("Field Centric")) {
            lastDpadDownState = true;
            currentDriveMode = "Field Centric";
            gamepad1.runRumbleEffect(new Gamepad.RumbleEffect.Builder().addStep(1.0, 1.0, 200).build());
            fieldCentricTargetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        } else if (!gamepad1.dpad_down) lastDpadDownState = false;

        if (!visionModeActive) {
            if (currentDriveMode.equals("Robot Centric")) RobotCentricDrive();
            else if (currentDriveMode.equals("Field Centric")) FieldCentricDrive();
        }
    }

    private void FieldCentricDrive() {
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        if (Math.abs(gamepad1.right_stick_x) <= 0.01) {
            if (!lockHeading) fieldCentricTargetHeading = currentHeading;
            lockHeading = true;
        } else lockHeading = false;

        if (lockHeading) {
            double headingError = fieldCentricTargetHeading - currentHeading;
            headingError = Math.IEEEremainder(headingError, 2 * Math.PI);
            headingCorrection = Math.abs(headingError) < Math.toRadians(2) ? 0 :
                    FieldCentricPIDController.calculate(headingError) + Math.signum(headingError) * fieldCentricHeadingKf;
        }

        double speed = 1.0 - (cubicTerm * Math.pow(gamepad1.right_trigger, 3) + linearTerm * gamepad1.right_trigger);
        speed = Math.max(speed, minimumDriveSpeed);

        double y = -gamepad1.left_stick_y * speed;
        double x = gamepad1.left_stick_x * speed;
        double rx = gamepad1.right_stick_x * speed;

        if (gamepad1.start) {
            imu.resetYaw();
            gamepad1.runRumbleEffect(new Gamepad.RumbleEffect.Builder().addStep(1.0, 1.0, 450).build());
            fieldCentricTargetHeading = 0;
            targetHeading = 0;
        }

        double rotX = x * Math.cos(-currentHeading) - y * Math.sin(-currentHeading);
        double rotY = x * Math.sin(-currentHeading) + y * Math.cos(-currentHeading);
        if (lockHeading) rx = headingCorrection;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double fl = (rotY + rotX + rx) / denominator;
        double bl = (rotY - rotX + rx) / denominator;
        double fr = (rotY - rotX - rx) / denominator;
        double br = (rotY + rotX - rx) / denominator;

        front_left_motor.setPower(fl);
        back_left_motor.setPower(bl);
        front_right_motor.setPower(fr);
        back_right_motor.setPower(br);
    }

    private void RobotCentricDrive() {
        double speed = 1.0 - (cubicTerm * Math.pow(gamepad1.right_trigger, 3) + linearTerm * gamepad1.right_trigger);
        speed = Math.max(speed, minimumDriveSpeed);

        double y = -gamepad1.left_stick_y * speed;
        double x = gamepad1.left_stick_x * speed;
        double rx = gamepad1.right_stick_x * speed;

        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))));
        front_left_motor.setPower(fl / max);
        back_left_motor.setPower(bl / max);
        front_right_motor.setPower(fr / max);
        back_right_motor.setPower(br / max);
    }

    private void BackgroundOpModeTasks() {
        FieldCentricPIDController.setPID(fieldCentricHeadingKp, fieldCentricHeadingKi, fieldCentricHeadingKd);
        headingLockPID.setPID(headingKp, headingKi, headingKd);
        armSubSystem.VerticalPIDFSlideControl();
        armSubSystem.HorizontalPIDFSlideControl();
        armSubSystem.updateServos();
        CommandScheduler.getInstance().run();
    }

    private void ArmPositionToggling() {
        if (gamepad1.left_stick_button) armSubSystem.OuttakeOpModeIdlePosition();
        if (gamepad2.left_stick_button) armSubSystem.IntakeOpModeIdlePosition();

        if (gamepad1.left_bumper) {
            armSubSystem.PositionForSpecimenCollection();
            lastLeftBumperState = true;
            releaseTimer.reset();
        } else if (lastLeftBumperState && releaseTimer.milliseconds() >= 20) {
            armSubSystem.PositionForSpecimenScoring();
            lastLeftBumperState = false;
        }

        if (gamepad1.a) armSubSystem.ScoreSpecimen();
        if (gamepad2.a) armSubSystem.ThrowSampleOutIntoObservationZone();

        boolean isHolding = gamepad2.left_trigger > 0.05;

        if (isHolding) {
            if (!wasHoldingLeftTrigger) {
                armSubSystem.PositionForSampleCollection();
                wasHoldingLeftTrigger = true;
            }

            double slideAdjust = -gamepad2.left_stick_x * 30;
            armSubSystem.horizontalSlideTargetPosition += slideAdjust;

            double rotationInput = -gamepad2.right_stick_x;
            if (Math.abs(rotationInput) > 0.01) {
                double current = armSubSystem.horizontalRotationServo.getTargetPosition();
                double adjusted = Math.max(0.0, Math.min(1.0, current + (rotationInput * 0.0105)));
                armSubSystem.horizontalRotationServo.setTargetPosition(adjusted);
            }

        } else if (wasHoldingLeftTrigger) {
            if (currentSampleCollectionMode.equals("Sample Mode")) {
                armSubSystem.SampleCollectionModeCollectSample();
                wasHoldingLeftTrigger = false;
            } else {
                armSubSystem.SpecimenCollectionModeCollectSample();
                wasHoldingLeftTrigger = false;
            }
        }

        if (gamepad1.b) armSubSystem.TransferSampleToOuttake();
        if (gamepad1.left_trigger > 0.05 && !wasScoringWithLeftTrigger) {
            armSubSystem.ScoreSampleInHighBasket();
            wasScoringWithLeftTrigger = true;
        } else if (gamepad1.left_trigger <= 0.05) {
            wasScoringWithLeftTrigger = false;
        }
    }

    private void SampleModeToggling() {
        if (gamepad2.dpad_up && !lastGamepad2DpadUpState && !currentDriveMode.equals("Sample Mode")) {
            lastGamepad2DpadUpState = true;
            currentSampleCollectionMode = "Sample Mode";
            gamepad2.runRumbleEffect(new Gamepad.RumbleEffect.Builder().addStep(1.0, 1.0, 200).build());
        } else if (!gamepad2.dpad_up) lastGamepad2DpadUpState = false;

        if (gamepad2.dpad_down && !lastGamepad2DpadDownState && !currentDriveMode.equals("Specimen Mode")) {
            lastGamepad2DpadDownState = true;
            currentSampleCollectionMode = "Specimen Mode";
            gamepad2.runRumbleEffect(new Gamepad.RumbleEffect.Builder().addStep(1.0, 1.0, 200).build());
        } else if (!gamepad2.dpad_down) lastGamepad2DpadDownState = false;
    }

    private void updateTelemetry() {
        telemetry.addData("Drive Mode", currentDriveMode);
        telemetry.addData("Collection Mode", currentSampleCollectionMode);
        telemetry.addData("Vision Mode", visionModeActive ? "ACTIVE (Press Y to exit)" : "INACTIVE (Press Y to activate)");

        if (visionModeActive) {
            telemetry.addData("Alignment", alignmentFinished ? "ALIGNED" : "ALIGNING");
            telemetry.addData("Heading Correction", getHeadingCorrection());
        }

        telemetry.update();
    }
}