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
import com.qualcomm.robotcore.hardware.Servo;
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

    /** Vision System Components **/
    private Limelight3A limelight;
    private boolean visionModeActive = false;
    private boolean lastVisionToggleState = false;

    // Vision parameters
    public static double IMAGE_CENTER_X = 320.0;
    public static double ALIGNMENT_OFFSET = -2.5;
    public static double FINAL_ALIGNMENT_TOLERANCE = 3;
    public static int REQUIRED_STABLE_FRAMES = 10;
    public static double MAX_STRAFE_POWER = 0.45;
    public static double MIN_STRAFE_POWER = 0.3;
    public static double STUCK_THRESHOLD = 0;
    public static double STUCK_ERROR_THRESHOLD = 0;

    // Vision PID constants
    public static double kP = 0.005;
    public static double kI = 0;
    public static double kD = 0.0002;
    public static double kF = 0.1;
    public static double STUCK_KP_BOOST = 0;
    public static double STUCK_KF_BOOST = 0;
    public static double CLOSE_RANGE_THRESHOLD = 0;
    public static double CLOSE_RANGE_KP = 0;

    // Arm parameters for vision
    public static double slideExtensionOffsetInches = 6.889764;
    public static double logCorrectionFactor = 18.2;
    public static double horizontalSlideTicksPerInch = 58;
    public static double ROTATION_OFFSET_DEGREES = 66;
    public static int SERVO_ROTATION_DELAY_MS = 600;

    // Vision state variables
    private PIDController alignmentPID;
    private double lastCorrectedWorldY = 0;
    private double smoothedAngle = 0;
    private double lockedAngle = 0;
    private double lockedSlideTarget = 0;
    private double lastError = 0;
    private double integralSum = 0;
    private ElapsedTime pidTimer = new ElapsedTime();
    private ElapsedTime stuckTimer = new ElapsedTime();
    private boolean isStuck = false;
    private boolean alignmentFinished = false;
    private boolean slideTargetLocked = false;
    private boolean waitingForSlideRetraction = false;
    private boolean rotationServoPositionLocked = false;
    private boolean collectionTriggered = false;
    private boolean servoRotationStarted = false;
    private long servoRotationStartTime = 0;
    private int stableFrameCount = 0;
    private double lastStableX = 0;

    /** Drive Control Variables **/
    private static final double minimumDriveSpeed = 0.3;
    public static double cubicTerm = 0.5;
    public static double linearTerm = 0.4;

    /** PID Variables **/
    public static double headingKp = 3;
    public static double headingKi = 0;
    public static double headingKd = 0.275;
    public static double headingKf = 0.05;

    /** Field Centric Stuff **/
    IMU imu;
    double targetHeading;
    double headingCorrection;
    PIDController FieldCentricPIDController;

    /** Booleans **/
    private boolean lastDpadUpState;
    private boolean lastDpadDownState;
    private boolean lastGamepad2DpadUpState;
    private boolean lastGamepad2DpadDownState;
    private boolean lockHeading = true;
    private boolean lastLeftBumperState = false;
    private boolean wasHoldingLeftTrigger = false;
    private boolean wasScoringWithLeftTrigger = false;

    ElapsedTime releaseTimer;

    /** Drive Mode + Game Mode Selection **/
    private String currentDriveMode = "Robot Centric";
    private String currentSampleCollectionMode = "Specimen Mode";

    public void runOpMode(){
        armSubSystem = new DougieArmSubSystem(hardwareMap);

        // Initialize drive system
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

        // Initialize vision system
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();

        alignmentPID = new PIDController(kP, kI, kD);
        alignmentPID.setTolerance(FINAL_ALIGNMENT_TOLERANCE);

        releaseTimer = new ElapsedTime();

        // Init position for the INTAKE + OUTTAKE for when the init button is pressed
        armSubSystem.IntakeInitIdlePosition();
        armSubSystem.OuttakeInitIdlePosition();

        CommandScheduler.getInstance().run();
        armSubSystem.updateServos();

        telemetry.addData("Status: ", "Ready to start");
        telemetry.update();

        waitForStart();

        // Init position for the INTAKE for when the start button is pressed
        armSubSystem.IntakeOpModeIdlePosition();
        armSubSystem.OuttakeOpModeIdlePosition();
        CommandScheduler.getInstance().run();
        armSubSystem.updateServos();
        armSubSystem.HorizontalPIDFSlideControl();

        releaseTimer.reset();
        releaseTimer.startTime();

        targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        pidTimer.reset();
        stuckTimer.reset();

        while(opModeIsActive()){

            if(gamepad1.y && !lastVisionToggleState) {
                visionModeActive = !visionModeActive;
                if(visionModeActive) {
                    resetVisionSystem();
                    Gamepad.RumbleEffect visionModeRumble = new Gamepad.RumbleEffect.Builder()
                            .addStep(1.0, 1.0, 150)
                            .build();
                    gamepad1.runRumbleEffect(visionModeRumble);
                }
            }
            lastVisionToggleState = gamepad1.y;

            if(visionModeActive) {
                handleVisionAlignment();
            } else {
                DriveModeToggling();
                SampleModeToggling();
                ArmPositionToggling();
            }

            BackgroundOpModeTasks();
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
        isStuck = false;
        armSubSystem.horizontalSlideTargetPosition = 0;
        armSubSystem.LimeLightIntakeIdlePosition();
        armSubSystem.PositionForSampleScanning();
        integralSum = 0;
        stuckTimer.reset();
    }

    private void handleVisionAlignment() {
        handleArmControl();

        if (!processVisionAndAlignment()) {
            return;
        }

        updateVisionTelemetry();
    }

    private boolean processVisionAndAlignment() {
        if (waitingForSlideRetraction) {
            return false;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || result.getPythonOutput() == null || result.getPythonOutput().length < 7) {
            telemetry.addLine("No target detected");
            stableFrameCount = 0;
            isStuck = false;
            applyMotorPowers(0, getHeadingCorrection());
            return false;
        }

        double[] output = result.getPythonOutput();
        boolean locked = output[0] == 1.0 || output[1] == 1.0 || output[2] == 1.0;
        if (!locked) {
            telemetry.addLine("No block locked");
            stableFrameCount = 0;
            isStuck = false;
            applyMotorPowers(0, getHeadingCorrection());
            return false;
        }

        double angle = output[3];
        double centerX = output[4];
        double worldY = output[6];
        double targetX = IMAGE_CENTER_X + ALIGNMENT_OFFSET;
        double error = centerX - targetX;

        // Calculate time step for derivative and integral terms
        double dt = pidTimer.seconds();
        pidTimer.reset();

        // Check if we're stuck
        if (Math.abs(error) > STUCK_ERROR_THRESHOLD && !isStuck) {
            if (stuckTimer.seconds() > STUCK_THRESHOLD) {
                isStuck = true;
                stuckTimer.reset();
            }
        } else {
            stuckTimer.reset();
            isStuck = false;
        }

        // Dynamic PID adjustment
        double currentKP = (Math.abs(error) < CLOSE_RANGE_THRESHOLD) ? CLOSE_RANGE_KP : kP;
        double currentKF = kF;

        // Boost PID when stuck
        if (isStuck) {
            currentKP += STUCK_KP_BOOST;
            currentKF += STUCK_KF_BOOST;
        }

        // Calculate PID terms
        double proportional = currentKP * error;

        // Integral term with anti-windup
        if (Math.abs(error) > FINAL_ALIGNMENT_TOLERANCE * 2) {
            integralSum += error * dt;
        } else {
            integralSum = 0;
        }
        double integral = kI * integralSum;

        // Derivative term
        double derivative = kD * ((error - lastError) / dt);
        lastError = error;

        // Calculate PID output
        double pidOutput = (proportional + integral + derivative);

        // Add feedforward term with boost if stuck
        double feedforward = Math.signum(error) * currentKF;
        double strafePower = pidOutput + feedforward;

        // Ensure minimum power to overcome static friction
        if (Math.abs(strafePower) > 0 && Math.abs(strafePower) < MIN_STRAFE_POWER) {
            strafePower = Math.copySign(MIN_STRAFE_POWER, strafePower);
        }

        // Limit maximum power
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
            isStuck = false;
        }

        // Smooth angle measurement
        smoothedAngle = 0.8 * smoothedAngle + 0.2 * angle;

        handleServoRotation();
        handleSlideExtension(worldY);
        handleCollectionSequence();

        applyMotorPowers(alignmentFinished ? 0 : strafePower, getHeadingCorrection());
        return true;
    }

    private void handleServoRotation() {
        if (!servoRotationStarted && stableFrameCount >= 3) {
            lockedAngle = ((smoothedAngle % 360) + 360) % 360;
            if (lockedAngle > 180) lockedAngle = 360 - lockedAngle;

            double finalAngle = lockedAngle + ROTATION_OFFSET_DEGREES;
            finalAngle = Math.max(0.0, Math.min(180.0, finalAngle));
            double mappedServoPosition = finalAngle / 180.0;
            armSubSystem.horizontalRotationServo.setTargetPosition(mappedServoPosition);

            servoRotationStarted = true;
            servoRotationStartTime = System.currentTimeMillis();
            rotationServoPositionLocked = true;
        }
    }

    private void handleSlideExtension(double worldY) {
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
    }

    private void handleCollectionSequence() {
        if (alignmentFinished && rotationServoPositionLocked && slideTargetLocked && !collectionTriggered) {
            long timeSinceServoRotation = System.currentTimeMillis() - servoRotationStartTime;
            if (timeSinceServoRotation >= SERVO_ROTATION_DELAY_MS) {
                armSubSystem.LimelightPositionForSampleCollection();
                sleep(300);
                armSubSystem.LimelightCollectSample();
                collectionTriggered = true;
            }
        }
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

        if (Math.abs(headingError) < Math.toRadians(1.5)) {
            return 0;
        }

        double correction = FieldCentricPIDController.calculate(headingError);
        double feedforward = Math.signum(headingError) * headingKf;
        return correction + feedforward;
    }

    private void updateVisionTelemetry() {
        telemetry.addData("Vision Mode", "ACTIVE (Press Y to exit)");
        telemetry.addData("Alignment Status", alignmentFinished ? "ALIGNED" : "ALIGNING");
        telemetry.addData("Error", lastError);
        telemetry.addData("Stable Frames", stableFrameCount);
        telemetry.addData("Heading Correction", getHeadingCorrection());
        telemetry.addData("Servo Angle", lockedAngle + ROTATION_OFFSET_DEGREES);
        telemetry.addData("Stuck Detection", isStuck ? "STUCK - BOOSTING" : "OK");
        telemetry.update();
    }

    private void handleArmControl() {
        armSubSystem.VerticalPIDFSlideControl();
        armSubSystem.HorizontalPIDFSlideControl();
        armSubSystem.updateServos();
        CommandScheduler.getInstance().run();

        if (waitingForSlideRetraction && Math.abs(armSubSystem.currentHorizontalSlidePosition) <= 50) {
            waitingForSlideRetraction = false;
        }

        if (waitingForSlideRetraction) {
            telemetry.addLine("Waiting for slide to retract...");
            applyMotorPowers(0, getHeadingCorrection());
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

            targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        } else if (!gamepad1.dpad_down) lastDpadDownState = false;

        if (!visionModeActive) {
            if (currentDriveMode.equals("Robot Centric")) RobotCentricDrive();
            else if (currentDriveMode.equals("Field Centric")) FieldCentricDrive();
        }
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

        double adjustedDrivingSpeed = cubicTerm * Math.pow(gamepad1.right_trigger, 3) + linearTerm * gamepad1.right_trigger;
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
        double adjustedDrivingSpeed = cubicTerm * Math.pow(gamepad1.right_trigger, 3) + linearTerm * gamepad1.right_trigger;
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
        armSubSystem.updateServos();

        CommandScheduler.getInstance().run();
    }

    private void ArmPositionToggling() {
        // Outtake and Intake Idle Positions
        if(gamepad1.left_stick_button) armSubSystem.OuttakeOpModeIdlePosition();
        if(gamepad2.left_stick_button) armSubSystem.IntakeOpModeIdlePosition();

        /** Specimen Actions **/
        if (gamepad1.left_bumper) {
            armSubSystem.PositionForSpecimenCollection();
            lastLeftBumperState = true;
            releaseTimer.reset();
        } else if (lastLeftBumperState && releaseTimer.milliseconds() >= 20){
                armSubSystem.PositionForSpecimenScoring();
                lastLeftBumperState = false;
        }

        if (gamepad1.a) armSubSystem.ScoreSpecimen();
        if(gamepad2.a) armSubSystem.ThrowSampleOutIntoObservationZone();

        /** Sample Actions **/
        boolean isCurrentlyHoldingLeftTrigger = gamepad2.left_trigger > 0.05;

        if(gamepad2.y && currentSampleCollectionMode.equals("Sample Mode")){
            armSubSystem.SlightlyDropSampleForTransfer();
        }

        if (isCurrentlyHoldingLeftTrigger) {
            if (!wasHoldingLeftTrigger) {
                armSubSystem.PositionForSampleCollection();
                wasHoldingLeftTrigger = true;
            }

            double sensitivity = 30;
            armSubSystem.horizontalSlideTargetPosition += -gamepad2.left_stick_x * sensitivity;

            double rotationSensitivity = 0.0105;
            double joystickInput = -gamepad2.right_stick_x;

            if (Math.abs(joystickInput) > 0.01) {
                double currentRotation = armSubSystem.horizontalRotationServo.getTargetPosition();
                double newRotation = currentRotation + (joystickInput * rotationSensitivity);
                newRotation = Math.max(0.0, Math.min(1.0, newRotation));
                armSubSystem.horizontalRotationServo.setTargetPosition(newRotation);
            }

        } else if (wasHoldingLeftTrigger) {
            if(currentSampleCollectionMode.equals("Sample Mode")){
                armSubSystem.SampleCollectionModeCollectSample();
                wasHoldingLeftTrigger = false;

                if(gamepad2.y) armSubSystem.SlightlyDropSampleForTransfer();
            }
            else if(currentSampleCollectionMode.equals("Specimen Mode")) {
                armSubSystem.SpecimenCollectionModeCollectSample();
                wasHoldingLeftTrigger = false;
            }
        }

        // Transfer to outtake
        if (gamepad1.b) armSubSystem.TransferSampleToOuttake();

        if (gamepad1.left_trigger > 0.05 && !wasScoringWithLeftTrigger) {
            armSubSystem.ScoreSampleInHighBasket();
            wasScoringWithLeftTrigger = true;
        } else if (gamepad1.left_trigger <= 0.05) {
            wasScoringWithLeftTrigger = false;
        }
    }

    private void SampleModeToggling(){
        // Toggling sample mode
        if (gamepad2.dpad_up && !lastGamepad2DpadUpState && !currentDriveMode.equals("Sample Mode")) {
            lastGamepad2DpadUpState = true;
            currentSampleCollectionMode = "Sample Mode";
            Gamepad.RumbleEffect sampleModeRumbleUpdate = new Gamepad.RumbleEffect.Builder()
                    .addStep(1.0, 1.0, 200)
                    .build();
            gamepad2.runRumbleEffect(sampleModeRumbleUpdate);
        } else if (!gamepad2.dpad_up) lastGamepad2DpadUpState = false;

        // Toggling specimen mode
        if (gamepad2.dpad_down && !lastGamepad2DpadDownState && !currentDriveMode.equals("Specimen Mode")) {
            lastGamepad2DpadDownState = true;
            currentSampleCollectionMode = "Specimen Mode";
            Gamepad.RumbleEffect sampleModeRumbleUpdate = new Gamepad.RumbleEffect.Builder()
                    .addStep(1.0, 1.0, 200)
                    .build();
            gamepad2.runRumbleEffect(sampleModeRumbleUpdate);
        } else if (!gamepad2.dpad_down) lastGamepad2DpadDownState = false;
    }
}