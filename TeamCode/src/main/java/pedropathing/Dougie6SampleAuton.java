package pedropathing;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedropathing.constants.FConstants;
import pedropathing.constants.LConstants;

@Autonomous(name = "4 Sample Auto")
public class Dougie6SampleAuton extends LinearOpMode {

    DougieArmSubSystem armSubSystem;
    private Follower follower;
    private Limelight3A limelight;

    // Vision parameters (same as in tele-op)
    public static double IMAGE_CENTER_X = 320.0;
    public static double ALIGNMENT_OFFSET = -9.9;
    public static double FINAL_ALIGNMENT_TOLERANCE = 10;
    public static int REQUIRED_STABLE_FRAMES = 8;
    public static double MAX_STRAFE_POWER = 0.45;
    public static double MIN_STRAFE_POWER = 0.08;
    public static double STUCK_THRESHOLD = 0.5;
    public static double STUCK_ERROR_THRESHOLD = 10.0;

    // Vision PID constants
    public static double kP = 0.00865;
    public static double kI = 0;
    public static double kD = 0.00225;
    public static double kF = 0.1;
    public static double STUCK_KP_BOOST = 0;
    public static double STUCK_KF_BOOST = 0;
    public static double CLOSE_RANGE_THRESHOLD = 0;
    public static double CLOSE_RANGE_KP = 0;

    // Arm parameters for vision
    public static double slideExtensionOffsetInches = 6.889764;
    public static double logCorrectionFactor = 18.25;
    public static double horizontalSlideTicksPerInch = 59;
    public static double ROTATION_OFFSET_DEGREES = 64.5;
    public static int SERVO_ROTATION_DELAY_MS = 500;

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

    // Starting Position of our robot
    private final Pose startPose = new Pose(10.5, 112, Math.toRadians(0));

    // Score preload
    private final Pose scoreSamplePreloadIntoHighBasket = new Pose(15.15, 123.15, Math.toRadians(-42));

    // Collect and score sample spike 1
    private final Pose collecting1stSampleSpike1 = new Pose(18, 125, Math.toRadians(-4.5));
    private final Pose scoring1stSampleIntoHighBasket = new Pose(16.5, 126.1, Math.toRadians(-42));

    // Collect and score sample spike 2
    private final Pose collecting2ndSampleSpike2 = new Pose(18, 126.25, Math.toRadians(14));
    private final Pose scoring2ndSampleIntoHighBasket = new Pose(16.5, 126.1, Math.toRadians(-42));

    // Collect and score sample spike 3
    private final Pose collecting3rdSampleSpike3 = new Pose(20, 133, Math.toRadians(27));
    private final Pose scoring3rdSampleIntoHighBasket = new Pose(16.5, 126.1, Math.toRadians(-42));

    private Path scoreSamplePreload;
    private Path collect1stSample;
    private PathChain collect1stSampleChain;
    private Path scoring1stSample;

    private Path collect2ndSample;
    private PathChain collect2ndSampleChain;
    private Path scoring2ndSample;

    private Path collect3rdSample;
    private PathChain collect3rdSampleChain;
    private Path scoring3rdSample;

    public void buildPaths() {
        /*** Scoring 1st Sample ***/
        scoreSamplePreload = new Path(new BezierLine(new Point(startPose), new Point(scoreSamplePreloadIntoHighBasket)));
        scoreSamplePreload.setLinearHeadingInterpolation(Math.toRadians(0), scoreSamplePreloadIntoHighBasket.getHeading());

        /*** Scoring 2nd Sample (Spike 1) ***/
        collect1stSample = new Path(new BezierLine(new Point(scoreSamplePreloadIntoHighBasket), new Point(collecting1stSampleSpike1)));
        collect1stSample.setLinearHeadingInterpolation(scoreSamplePreloadIntoHighBasket.getHeading(), collecting1stSampleSpike1.getHeading());
        collect1stSample.setZeroPowerAccelerationMultiplier(1);
        collect1stSample.setPathEndTimeoutConstraint(350);

        collect1stSampleChain = new PathChain(collect1stSample);

        scoring1stSample = new Path(new BezierLine(new Point(collecting1stSampleSpike1), new Point(scoring1stSampleIntoHighBasket)));
        scoring1stSample.setLinearHeadingInterpolation(Math.toRadians(0), scoring1stSampleIntoHighBasket.getHeading());
        scoring1stSample.setZeroPowerAccelerationMultiplier(1);

        /*** Scoring 3rd Sample (Spike 2) ***/
        collect2ndSample = new Path(new BezierLine(new Point(scoring1stSampleIntoHighBasket), new Point(collecting2ndSampleSpike2)));
        collect2ndSample.setLinearHeadingInterpolation(scoring1stSampleIntoHighBasket.getHeading(), collecting2ndSampleSpike2.getHeading());
        collect2ndSample.setZeroPowerAccelerationMultiplier(1);
        collect2ndSample.setPathEndTimeoutConstraint(300);

        collect2ndSampleChain = new PathChain(collect2ndSample);

        scoring2ndSample = new Path(new BezierLine(new Point(collecting2ndSampleSpike2), new Point(scoring2ndSampleIntoHighBasket)));
        scoring2ndSample.setLinearHeadingInterpolation(collecting2ndSampleSpike2.getHeading(), scoring2ndSampleIntoHighBasket.getHeading());

        /*** Scoring 4th Sample (Spike 2) ***/
        collect3rdSample = new Path(new BezierLine(new Point(scoring2ndSampleIntoHighBasket), new Point(collecting3rdSampleSpike3)));
        collect3rdSample.setLinearHeadingInterpolation(scoring2ndSampleIntoHighBasket.getHeading(), collecting3rdSampleSpike3.getHeading());
        collect3rdSample.setZeroPowerAccelerationMultiplier(0.25);
        collect3rdSample.setPathEndTimeoutConstraint(300);

        collect3rdSampleChain = new PathChain(collect3rdSample);

        scoring3rdSample = new Path(new BezierLine(new Point(collecting3rdSampleSpike3), new Point(scoring3rdSampleIntoHighBasket)));
        scoring3rdSample.setLinearHeadingInterpolation(collecting3rdSampleSpike3.getHeading(), scoring3rdSampleIntoHighBasket.getHeading());
    }

    public void runOpMode() {
        buildPaths();
        armSubSystem = new DougieArmSubSystem(hardwareMap);

        // Initialize vision system
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();

        alignmentPID = new PIDController(kP, kI, kD);
        alignmentPID.setTolerance(FINAL_ALIGNMENT_TOLERANCE);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();

        // Building Autonomous Route
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        /** Scoring sample preload **/
                        new InstantCommand(() -> armSubSystem.PositionForHighBucketScoring()),
                        new WaitCommand(100),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> armSubSystem.AutonomousPositionForSampleCollection(985, 0.24)),
                                new WaitCommand(1250)
                        ),
                        new FollowPath(follower, scoreSamplePreload),
                        new WaitCommand(300),
                        new InstantCommand(() -> armSubSystem.ScoreSampleInHighBasket()),
                        new WaitCommand(350),

                        /** Collecting and Scoring spike 1 Sample **/
                        new ParallelCommandGroup(
                                new FollowPath(follower, collect1stSampleChain, true),
                                new InstantCommand(() -> armSubSystem.AutonomousPositionForSampleCollection(1560, 0.25)),
                                new WaitCommand(900)
                        ),
                        new InstantCommand(() -> armSubSystem.SampleCollectionModeCollectSample()),
                        new WaitCommand((2500)),
                        new InstantCommand(() -> armSubSystem.TransferSampleToOuttake()),
                        new WaitCommand(1500),
                        new FollowPath(follower, scoring1stSample),

                        new WaitCommand(400),
                        new InstantCommand(() -> armSubSystem.ScoreSampleInHighBasket()),


                        /** Collecting and Scoring spike 2 Sample **/
                        new ParallelCommandGroup(
                                new FollowPath(follower, collect2ndSampleChain, true),
                                new InstantCommand(() -> armSubSystem.AutonomousPositionForSampleCollection(1350, 0.25)),
                                new WaitCommand(900)
                        ),
                        new InstantCommand(() -> armSubSystem.SampleCollectionModeCollectSample()),
                        new WaitCommand((2500)),
                        new InstantCommand(() -> armSubSystem.TransferSampleToOuttake()),
                        new WaitCommand(1500),
                        new FollowPath(follower, scoring2ndSample),

                        new WaitCommand(400),
                        new InstantCommand(() -> armSubSystem.ScoreSampleInHighBasket()),

                        /** Collecting and Scoring spike 3rd Sample **/
                        new ParallelCommandGroup(
                                new FollowPath(follower, collect3rdSampleChain, true),
                                new InstantCommand(() -> armSubSystem.AutonomousPositionForSampleCollection(1600, 0.265)),
                                new WaitCommand(900)
                        ),
                        new InstantCommand(() -> armSubSystem.SampleCollectionModeCollectSample()),
                        new WaitCommand((2500)),
                        new InstantCommand(() -> armSubSystem.TransferSampleToOuttake()),
                        new WaitCommand(1500),
                        new FollowPath(follower, scoring2ndSample),

                        new WaitCommand(400),
                        new InstantCommand(() -> armSubSystem.ScoreSampleInHighBasket())
                )
        );

        telemetry.addData("Status: ", "Ready to start...");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            CommandScheduler.getInstance().run();
            follower.update();

            armSubSystem.VerticalPIDFSlideControl();
            armSubSystem.HorizontalPIDFSlideControl();
            armSubSystem.updateServos();
        }
    }

    private void performVisionAlignmentAndCollection() {
        resetVisionSystem();
        armSubSystem.PositionForSampleScanning();

        // Vision alignment loop
        while (opModeIsActive() && !processVisionAndAlignment()) {
            // Update robot systems while aligning
            CommandScheduler.getInstance().run();
            follower.update();
            armSubSystem.VerticalPIDFSlideControl();
            armSubSystem.HorizontalPIDFSlideControl();
            armSubSystem.updateServos();
        }

        // Wait for collection to complete
        while (opModeIsActive() && !collectionTriggered) {
            CommandScheduler.getInstance().run();
            follower.update();
            armSubSystem.VerticalPIDFSlideControl();
            armSubSystem.HorizontalPIDFSlideControl();
            armSubSystem.updateServos();
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
        pidTimer.reset();
    }

    private boolean processVisionAndAlignment() {
        if (waitingForSlideRetraction) {
            if (Math.abs(armSubSystem.currentHorizontalSlidePosition) <= 50) {
                waitingForSlideRetraction = false;
            }
            return false;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || result.getPythonOutput() == null || result.getPythonOutput().length < 7) {
            telemetry.addLine("No target detected");
            stableFrameCount = 0;
            isStuck = false;
            applyMotorPowers(0, 0);
            return false;
        }

        double[] output = result.getPythonOutput();
        boolean locked = output[0] == 1.0 || output[1] == 1.0 || output[2] == 1.0;
        if (!locked) {
            telemetry.addLine("No block locked");
            stableFrameCount = 0;
            isStuck = false;
            applyMotorPowers(0, 0);
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
        double integral = kI * integralSum;
        double derivative = kD * ((error - lastError) / dt);
        lastError = error;

        // Calculate PID output
        double pidOutput = (proportional + integral + derivative);
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

        applyMotorPowers(alignmentFinished ? 0 : strafePower, 0);

        telemetry.addData("Vision Alignment", alignmentFinished ? "ALIGNED" : "ALIGNING");
        telemetry.addData("Error", error);
        telemetry.addData("Stable Frames", stableFrameCount);
        telemetry.update();

        return collectionTriggered;
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

        /*
        follower.getFrontLeftMotor().setPower(fl / max);
        follower.getFrontRightMotor().setPower(fr / max);
        follower.getBackLeftMotor().setPower(bl / max);
        follower.getBackRightMotor().setPower(br / max);

         */
    }
}