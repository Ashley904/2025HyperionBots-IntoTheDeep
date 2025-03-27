package pedropathing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Tune INCHES_PER_PIXEL + Threshold", group = "Vision")
public class TuneLimelightValues extends LinearOpMode {

    private Limelight3A limelight;

    private static final double IMAGE_WIDTH = 640.0;
    private static final double IMAGE_CENTER_X = IMAGE_WIDTH / 2.0;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();

        telemetry.addLine("Alignment Tuner Ready — Move robot and observe output");
        telemetry.addLine("Use this to determine INCHES_PER_PIXEL + OFFSET_THRESHOLD");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null) {
                double[] output = result.getPythonOutput();

                if (output != null && output.length >= 6) {
                    boolean lockedYellow = output[0] == 1.0;
                    boolean lockedRed = output[1] == 1.0;
                    boolean lockedBlue = output[2] == 1.0;

                    boolean targetLocked = lockedYellow || lockedRed || lockedBlue;
                    double centerX = output[4];
                    double offsetX = centerX - IMAGE_CENTER_X;

                    telemetry.addData("Target Locked", targetLocked);
                    telemetry.addData("Locked Color", lockedYellow ? "YELLOW" : lockedRed ? "RED" : lockedBlue ? "BLUE" : "NONE");
                    telemetry.addData("Center X (px)", centerX);
                    telemetry.addData("Offset from Center (px)", offsetX);

                    telemetry.addLine("======== Instructions ========");
                    telemetry.addLine("1. Move the robot known distances (e.g. 6 inches left of sample)");
                    telemetry.addLine("2. Record the reported offset in pixels");
                    telemetry.addLine("3. Use:  inches_per_pixel = known_inches / offsetX");
                    telemetry.addLine("4. Try different distances and average results");
                } else {
                    telemetry.addLine("[ERROR] No valid SnapScript output");
                }
            } else {
                telemetry.addLine("[ERROR] No Limelight result");
            }

            telemetry.update();
        }
    }
}
