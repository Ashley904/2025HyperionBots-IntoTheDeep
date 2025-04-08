package pedropathing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "Dougie Auto Align + Slide + Heading Lock")
public class DougieLimeLightVision extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private Servo rotationServo;

    DougieArmSubSystem armSubSystem;

    // Vision parameters
    public static double IMAGE_CENTER_X = 320.0;
    public static double ALIGNMENT_OFFSET = -2.5;
    public static double FINAL_ALIGNMENT_TOLERANCE = 3;
    public static int REQUIRED_STABLE_FRAMES = 3;
    public static double MAX_STRAFE_POWER = 0.45;
    public static double MIN_STRAFE_POWER = 0.3;
    public static double STUCK_THRESHOLD = 0;
    public static double STUCK_ERROR_THRESHOLD = 0;

    // PID constants with feedforward
    public static double kP = 0.005;
    public static double kI = 0;
    public static double kD = 0.0002;
    public static double kF = 0.1;
    public static double STUCK_KP_BOOST = 0;
    public static double STUCK_KF_BOOST = 0;

    // Dynamic PID scaling
    public static double CLOSE_RANGE_THRESHOLD = 0;
    public static double CLOSE_RANGE_KP = 0;

    // Heading control
    public static double headingKp = 3;
    public static double headingKi = 0;
    public static double headingKd = 0.275;
    public static double headingKf = 0.05;

    // Arm parameters
    public static double slideExtensionOffsetInches = 6.889764;
    public static double logCorrectionFactor = 18.25;
    public static double horizontalSlideTicksPerInch = 59;
    public static double ROTATION_OFFSET_DEGREES = 66;
    public static int SERVO_ROTATION_DELAY_MS = 500;

    // Controllers
    private PIDController alignmentPID;
    private PIDController headingLockPID;

    // State variables
    private double targetHeading = 0;
    private double lastCorrectedWorldY = 0;
    private double smoothedAngle = 0;
    private double lockedAngle = 0;
    private double lockedSlideTarget = 0;
    private double lastError = 0;
    private double lastHeadingCorrection = 0;
    private double integralSum = 0;
    private ElapsedTime pidTimer = new ElapsedTime();
    private ElapsedTime stuckTimer = new ElapsedTime();
    private boolean isStuck = false;

    // Status flags
    private boolean alignmentFinished = false;
    private boolean slideTargetLocked = false;
    private boolean trianglePressedLastLoop = false;
    private boolean waitingForSlideRetraction = false;
    private boolean rotationServoPositionLocked = false;
    private boolean collectionTriggered = false;
    private boolean servoRotationStarted = false;
    private long servoRotationStartTime = 0;

    // Alignment tracking
    private int stableFrameCount = 0;
    private double lastStableX = 0;

    @Override
    public void runOpMode() {
        initializeHardware();
        telemetry.addLine("Ready to align...");
        telemetry.update();
        waitForStart();

        targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        pidTimer.reset();
        stuckTimer.reset();

        while (opModeIsActive()) {
            handleResetSequence();
            handleArmControl();

            if (!processVisionAndAlignment()) {
                continue;
            }

            updateTelemetry();
        }
    }

    private void initializeHardware() {
        rotationServo = hardwareMap.get(Servo.class, "horizontalGripperRotation");
        rotationServo.setDirection(Servo.Direction.REVERSE);

        frontLeft = hardwareMap.dcMotor.get("FL");
        frontRight = hardwareMap.dcMotor.get("FR");
        backLeft = hardwareMap.dcMotor.get("BL");
        backRight = hardwareMap.dcMotor.get("BR");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();

        alignmentPID = new PIDController(kP, kI, kD);
        alignmentPID.setTolerance(FINAL_ALIGNMENT_TOLERANCE);

        headingLockPID = new PIDController(headingKp, headingKi, headingKd);
        headingLockPID.setTolerance(Math.toRadians(1.5));

        armSubSystem = new DougieArmSubSystem(hardwareMap);
    }

    private void handleResetSequence() {
        boolean trianglePressed = gamepad1.y;
        if (trianglePressed && !trianglePressedLastLoop) {
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
        trianglePressedLastLoop = trianglePressed;
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
            telemetry.update();
        }
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

        double dt = pidTimer.seconds();
        pidTimer.reset();

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

        if (isStuck) {
            currentKP += STUCK_KP_BOOST;
            currentKF += STUCK_KF_BOOST;
        }

        double proportional = currentKP * error;

        if (Math.abs(error) > FINAL_ALIGNMENT_TOLERANCE * 2) {
            integralSum += error * dt;
        } else {
            integralSum = 0;
        }
        double integral = kI * integralSum;

        double derivative = kD * ((error - lastError) / dt);
        lastError = error;

        double pidOutput = (proportional + integral + derivative);

        double feedforward = Math.signum(error) * currentKF;
        double strafePower = pidOutput + feedforward;

        if (Math.abs(strafePower) > 0 && Math.abs(strafePower) < MIN_STRAFE_POWER) {
            strafePower = Math.copySign(MIN_STRAFE_POWER, strafePower);
        }

        strafePower = Math.max(-MAX_STRAFE_POWER, Math.min(MAX_STRAFE_POWER, strafePower));


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

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    private double getHeadingCorrection() {
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double headingError = targetHeading - currentHeading;
        headingError = Math.IEEEremainder(headingError, 2 * Math.PI);

        if (Math.abs(headingError) < Math.toRadians(1.5)) {
            return 0;
        }

        double correction = headingLockPID.calculate(headingError);
        double feedforward = Math.signum(headingError) * headingKf;
        return correction + feedforward;
    }

    private void updateTelemetry() {
        telemetry.addData("Alignment Status", alignmentFinished ? "ALIGNED" : "ALIGNING");
        telemetry.addData("Error", lastError);
        telemetry.addData("Stable Frames", stableFrameCount);
        telemetry.addData("Heading Correction", getHeadingCorrection());
        telemetry.addData("Servo Angle", lockedAngle + ROTATION_OFFSET_DEGREES);
        telemetry.addData("Stuck Detection", isStuck ? "STUCK - BOOSTING" : "OK");
        telemetry.update();
    }
}