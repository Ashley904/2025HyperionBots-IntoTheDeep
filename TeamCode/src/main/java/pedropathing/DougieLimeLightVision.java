package pedropathing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import pedropathing.constants.FConstants;
import pedropathing.constants.LConstants;

@Config
@TeleOp(name = "Pure Lateral Align to Sample")
public class DougieLimeLightVision extends LinearOpMode {

    private Limelight3A limelight;
    private Follower follower;
    private DougieArmSubSystem armSubSystem;

    public static double PIXEL_TO_INCHES = 0.0625; // Tune this empirically
    public static double ALIGNMENT_TOLERANCE_INCHES = 0.5; // How close we need to be

    private boolean trianglePressedLastLoop = false;
    private boolean sampleDataSaved = false;
    private double savedCenterX = 0;

    @Override
    public void runOpMode() {
        // Hardware initialization
        Servo rotationServo = hardwareMap.get(Servo.class, "horizontalGripperRotation");
        rotationServo.setDirection(Servo.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        armSubSystem = new DougieArmSubSystem(hardwareMap);

        telemetry.addLine("Ready for pure lateral alignment...");
        telemetry.update();
        waitForStart();

        boolean pathStarted = false;

        while (opModeIsActive()) {
            boolean trianglePressed = gamepad1.y;

            // Reset system when Y is pressed
            if (trianglePressed && !trianglePressedLastLoop) {
                sampleDataSaved = false;
                pathStarted = false;
                armSubSystem.horizontalSlideTargetPosition = 0;
                armSubSystem.LimeLightIntakeIdlePosition();
                armSubSystem.PositionForSampleScanning();
            }
            trianglePressedLastLoop = trianglePressed;

            // Run subsystems
            armSubSystem.VerticalPIDFSlideControl();
            armSubSystem.HorizontalPIDFSlideControl();
            armSubSystem.updateServos();
            CommandScheduler.getInstance().run();

            // Vision processing (runs once per Y press)
            if (!sampleDataSaved && trianglePressed) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.getPythonOutput() != null && result.getPythonOutput().length >= 7) {
                    double[] output = result.getPythonOutput();
                    boolean locked = output[0] == 1.0 || output[1] == 1.0 || output[2] == 1.0;

                    if (locked) {
                        savedCenterX = output[4];
                        sampleDataSaved = true;

                        Pose robotPose = follower.getPose();

                        // Calculate pure lateral movement needed
                        double pixelOffset = savedCenterX - 320; // 320 = camera center
                        double lateralInches = pixelOffset * PIXEL_TO_INCHES;

                        // Target keeps same X and heading, only changes Y
                        Point startPoint = new Point(robotPose.getX(), robotPose.getY());
                        Point endPoint = new Point(robotPose.getX() + + lateralInches, robotPose.getY());

                        // Create a straight-line path for pure lateral movement
                        PathChain alignPath = follower.pathBuilder()
                                .addPath(new BezierLine(startPoint, endPoint))
                                .build();

                        follower.followPath(alignPath);
                        pathStarted = true;

                        telemetry.addData("Status", "Pure Lateral Alignment");
                        telemetry.addData("Current Pose", "X: %.2f, Y: %.2f", robotPose.getX(), robotPose.getY());
                        telemetry.addData("Lateral Move (in)", "%.2f", lateralInches);
                        telemetry.update();
                    }
                }
            }

            // Update follower if alignment is in progress
            if (pathStarted) {
                follower.update();

                // Check for completion
                Pose currentPose = follower.getPose();
                double currentOffset = (savedCenterX - 320) * PIXEL_TO_INCHES;
                if (Math.abs(currentOffset) < ALIGNMENT_TOLERANCE_INCHES) {
                    telemetry.addLine("Lateral Alignment Complete!");
                    telemetry.update();
                }
            }
        }
    }
}