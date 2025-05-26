package pedropathing.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedropathing.DougieArmSubSystem;

@Config
@TeleOp(group = "Tuners", name ="Autonomous Config OpMode")
public class AutonomousConfig extends LinearOpMode {

    Follower follower;
    DougieArmSubSystem armSubSystem;


    public static double pointX;
    public static double pointY;
    public static double controlPointX;
    public static double controlPointY;

    public static boolean bezierCurve = false;
    public static boolean bezierLine = true;

    /** Outtake Position Togglers **/
    public static boolean outtakeIdlePosition = false;
    public static boolean outtakeCollectionPosition = true;
    public static boolean outtakeScoringPosition = false;
    public static boolean outtakeScoreSpecimen = false;

    public static boolean followPath = false;


    /** Paths **/
    Path desiredBezierLinePath;
    Path desiredBezierCurvePath;

    Pose startingPose = new Pose(10.75, 10.75, 0);


    public void runOpMode(){

        follower.setStartingPose(startingPose);
        armSubSystem = new DougieArmSubSystem(hardwareMap);

        telemetry.addData("Status: ", "Ready To Start...");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            if(bezierLine){

                Pose desiredPose = new Pose(pointX, pointY, Math.toRadians(0));
                desiredBezierLinePath = new Path(new BezierLine(new Point(follower.getPose()), new Point(desiredPose)));
                desiredBezierLinePath.setConstantHeadingInterpolation(Math.toRadians(0));

            }else if (bezierCurve){

                Point bezierControlPoint = new Point(controlPointX, controlPointY);
                Pose desiredPose = new Pose(pointX, pointY, Math.toRadians(0));

                desiredBezierCurvePath = new Path(new BezierCurve(new Point(follower.getPose()), bezierControlPoint, new Point(desiredPose)));
                desiredBezierCurvePath.setConstantHeadingInterpolation(Math.toRadians(0));

            }

            /** Controlling Outtake **/
            switch(getCurrentOuttakePosition()){
                case 0: armSubSystem.OuttakeOpModeIdlePosition(); break;
                case 1: armSubSystem.PositionForSpecimenCollection(); break;
                case 2: armSubSystem.PositionForSpecimenScoring(); break;
                case 3: armSubSystem.ScoreSpecimen(); break;
            }

            armSubSystem.updateServos();
            armSubSystem.HorizontalPIDFSlideControl();
            armSubSystem.VerticalPIDFSlideControl();

            CommandScheduler.getInstance().run();
            if(followPath) follower.update();
        }
    }

    private int getCurrentOuttakePosition(){
        if(outtakeIdlePosition) return 0;
        else if(outtakeCollectionPosition) return 1;
        else if (outtakeScoringPosition) return 2;
        else if (outtakeScoreSpecimen) return 3;

        else return -1;
    }

}
