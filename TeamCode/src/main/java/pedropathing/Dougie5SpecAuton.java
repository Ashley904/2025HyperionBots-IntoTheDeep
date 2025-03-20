package pedropathing;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import pedropathing.constants.FConstants;
import pedropathing.constants.LConstants;

@Autonomous(name = "5 Specimen Auton")
public class Dougie5SpecAuton extends LinearOpMode {

    DougieArmSubSystem verticalArmSubSystem;

    private Follower follower;

    /**
     * Starting Position of our robot
     */
    private final Pose startPose = new Pose(10.5, 60, Math.toRadians(0));

    /**
     * Scoring the 1st specimen onto the high bar
     */
    private final Pose scoreSpecimenPreload1 = new Pose(39, 75, Math.toRadians(0));

    /**
     * Pushing the 1st sample into the observation zone
     */
    private final Pose push1stSampleIntoObservationZone1 = new Pose(50, 35.5, Math.toRadians(0));
    private final Pose push1stSampleIntoObservationZone2 = new Pose(57, 26.5, Math.toRadians(0));
    private final Pose push1stSampleIntoObservationZone3 = new Pose(32.5, 26.5, Math.toRadians(0));

    /**
     * Pushing the 2nd sample into the observation zone
     */
    private final Pose push2ndSampleIntoObservationZone1 = new Pose(50, 26, Math.toRadians(0));
    private final Pose push2ndSampleIntoObservationZone2 = new Pose(57, 17, Math.toRadians(0));
    private final Pose push2ndSampleIntoObservationZone3 = new Pose(27.5, 18, Math.toRadians(0));

    /**
     * Pushing the 3rd sample into the observation zone
     */
    private final Pose push3rdSampleIntoObservationZone1 = new Pose(50, 18, Math.toRadians(0));
    private final Pose push3rdSampleIntoObservationZone2 = new Pose(58, 9.25, Math.toRadians(0));
    private final Pose push3rdSampleIntoObservationZone3 = new Pose(14.2, 9.25, Math.toRadians(0));

    /**
     * Hanging the 2nd specimen onto the high bar
     */
    private final Pose hang2ndSpecimenOntoHighBar1 = new Pose(42.5, 70, Math.toRadians(0));
    private final Pose hang2ndSpecimenOntoHighBar2 = new Pose(43.15, 74, Math.toRadians(0));

    /**
     * Hanging the 3rd specimen onto the high bar
     */
    private final Pose hang3rdSpecimenOntoHighBar1 = new Pose(20, 35, Math.toRadians(0));
    private final Pose hang3rdSpecimenOntoHighBar2 = new Pose(15.15, 35, Math.toRadians(0));
    private final Pose hang3rdSpecimenOntoHighBar3 = new Pose(41.5, 70, Math.toRadians(0));
    private final Pose hang3rdSpecimenOntoHighBar4 = new Pose(42.5, 74, Math.toRadians(0));

    /**
     * Hanging the 4th specimen onto the high bar
     */
    private final Pose hang4thSpecimenOntoHighBar1 = new Pose(20, 35, Math.toRadians(0));
    private final Pose hang4thSpecimenOntoHighBar2 = new Pose(15.15, 35, Math.toRadians(0));
    private final Pose hang4thSpecimenOntoHighBar3 = new Pose(41.5, 70, Math.toRadians(0));
    private final Pose hang4thSpecimenOntoHighBar4 = new Pose(42.5, 74, Math.toRadians(0));

    /**
     * Hanging the 5th specimen onto the high bar
     */
    private final Pose hang5thSpecimenOntoHighBar1 = new Pose(20, 35, Math.toRadians(0));
    private final Pose hang5thSpecimenOntoHighBar2 = new Pose(15.15, 35, Math.toRadians(0));
    private final Pose hang5thSpecimenOntoHighBar3 = new Pose(42.5, 70, Math.toRadians(0));


    private Path scorePreload1;
    private Path scorePreload2;

    private Path push1stSample1;
    private Path push1stSample2;
    private Path push1stSample3;
    private PathChain chained1stSamplePush;

    private Path push2ndSample1;
    private Path push2ndSample2;
    private Path push2ndSample3;
    private PathChain chained2ndSamplePush;

    private Path push3rdSample1;
    private Path push3rdSample2;
    private Path push3rdSample3;
    private PathChain chained3rdSamplePush;

    private Path hang2ndSpecimen1;
    private Path hang2ndSpecimen2;

    private Path hang3rdSpecimen1;
    private Path hang3rdSpecimen2;
    private Path hang3rdSpecimen3;
    private Path hang3rdSpecimen4;

    private Path hang4thSpecimen1;
    private Path hang4thSpecimen2;
    private Path hang4thSpecimen3;
    private Path hang4thSpecimen4;

    private Path hang5thSpecimen1;
    private Path hang5thSpecimen2;
    private Path hang5thSpecimen3;

    public void buildPaths() {
        /*** Scoring 1st Specimen ***/
        scorePreload1 = new Path(new BezierLine(new Point(startPose), new Point(scoreSpecimenPreload1)));
        scorePreload1.setConstantHeadingInterpolation(Math.toRadians(0));



        /*** Pushing 1st Sample Into Observation Zone ***/
        Point controlPoint1 = new Point(15, 40);
        push1stSample1 = new Path(new BezierCurve(new Point(scoreSpecimenPreload1), controlPoint1, new Point(push1stSampleIntoObservationZone1)));
        push1stSample1.setConstantHeadingInterpolation(Math.toRadians(0));

        Point controlPoint2 = new Point(56.5, 30);
        push1stSample2 = new Path(new BezierCurve(new Point(push1stSampleIntoObservationZone1), controlPoint2, new Point(push1stSampleIntoObservationZone2)));
        push1stSample2.setConstantHeadingInterpolation(Math.toRadians(0));

        push1stSample3 = new Path(new BezierCurve(new Point(push1stSampleIntoObservationZone2), new Point(push1stSampleIntoObservationZone3)));
        push1stSample3.setConstantHeadingInterpolation(Math.toRadians(0));
        push1stSample3.setPathEndTimeoutConstraint(0);
        push1stSample3.setZeroPowerAccelerationMultiplier(10);

        chained1stSamplePush = new PathChain(push1stSample1, push1stSample2, push1stSample3);



        /*** Pushing 2nd Sample Into Observation Zone ***/
        push2ndSample1 = new Path(new BezierLine(new Point(push1stSampleIntoObservationZone3), new Point(push2ndSampleIntoObservationZone1)));
        push2ndSample1.setConstantHeadingInterpolation(Math.toRadians(0));
        push2ndSample1.setPathEndTimeoutConstraint(0);
        push2ndSample1.setZeroPowerAccelerationMultiplier(6.5);

        Point controlPoint4 = new Point(56.5, 20);
        push2ndSample2 = new Path(new BezierCurve(new Point(push2ndSampleIntoObservationZone1), controlPoint4, new Point(push2ndSampleIntoObservationZone2)));
        push2ndSample2.setConstantHeadingInterpolation(Math.toRadians(0));

        push2ndSample3 = new Path(new BezierLine(new Point(push2ndSampleIntoObservationZone2), new Point(push2ndSampleIntoObservationZone3)));
        push2ndSample3.setConstantHeadingInterpolation(Math.toRadians(0));
        push2ndSample3.setPathEndTimeoutConstraint(5);
        push2ndSample3.setZeroPowerAccelerationMultiplier(8);

        chained2ndSamplePush = new PathChain(push2ndSample1, push2ndSample2, push2ndSample3);

        /*** Pushing 3rd Sample Into Observation Zone ***/
        push3rdSample1 = new Path(new BezierLine(new Point(push2ndSampleIntoObservationZone3), new Point(push3rdSampleIntoObservationZone1)));
        push3rdSample1.setConstantHeadingInterpolation(Math.toRadians(0));

        Point controlPoint5 = new Point(60, 15);
        push3rdSample2 = new Path(new BezierCurve(new Point(push3rdSampleIntoObservationZone1), controlPoint5, new Point(push3rdSampleIntoObservationZone2)));
        push3rdSample2.setConstantHeadingInterpolation(Math.toRadians(0));

        push3rdSample3 = new Path(new BezierLine(new Point(push3rdSampleIntoObservationZone2), new Point(push3rdSampleIntoObservationZone3)));
        push3rdSample3.setConstantHeadingInterpolation(Math.toRadians(0));
        push3rdSample3.setPathEndTimeoutConstraint(100);
        push3rdSample3.setZeroPowerAccelerationMultiplier(6);

        chained3rdSamplePush = new PathChain(push3rdSample1, push3rdSample2, push3rdSample3);

        /*** Hanging 2nd Specimen Onto High Bar ***/
        Point hangControlPoint1 = new Point(12, 50);
        hang2ndSpecimen1 = new Path(new BezierCurve(new Point(push3rdSampleIntoObservationZone3), hangControlPoint1, new Point(hang2ndSpecimenOntoHighBar1)));
        hang2ndSpecimen1.setConstantHeadingInterpolation(Math.toRadians(0));
        hang2ndSpecimen1.setZeroPowerAccelerationMultiplier(6.5);

        hang2ndSpecimen2 = new Path(new BezierLine(new Point(hang2ndSpecimenOntoHighBar1), new Point(hang2ndSpecimenOntoHighBar2)));
        hang2ndSpecimen2.setConstantHeadingInterpolation(Math.toRadians(0));

        /*** Hanging 3rd Specimen Onto High Bar ***/

        Point hangingSpecimenControlPoint1 = new Point(22, 75);
        hang3rdSpecimen1 = new Path(new BezierCurve(new Point(hang2ndSpecimenOntoHighBar2), hangingSpecimenControlPoint1, new Point(hang3rdSpecimenOntoHighBar1)));
        hang3rdSpecimen1.setConstantHeadingInterpolation(Math.toRadians(0));

        hang3rdSpecimen2 = new Path(new BezierLine(new Point(hang3rdSpecimenOntoHighBar1), new Point(hang3rdSpecimenOntoHighBar2)));
        hang3rdSpecimen2.setConstantHeadingInterpolation(Math.toRadians(0));

        Point hangingSpecimenControlPoint2 = new Point(25, 65);
        hang3rdSpecimen3 = new Path(new BezierCurve(new Point(hang3rdSpecimenOntoHighBar2), hangingSpecimenControlPoint2, new Point(hang3rdSpecimenOntoHighBar3)));
        hang3rdSpecimen3.setConstantHeadingInterpolation(Math.toRadians(0));

        hang3rdSpecimen4 = new Path(new BezierLine(new Point(hang3rdSpecimenOntoHighBar3), new Point(hang3rdSpecimenOntoHighBar4)));
        hang3rdSpecimen4.setConstantHeadingInterpolation(Math.toRadians(0));

        /*** Hanging 4th Specimen Onto High Bar ***/
        hang4thSpecimen1 = new Path(new BezierCurve(new Point(hang3rdSpecimenOntoHighBar4), hangingSpecimenControlPoint1, new Point(hang4thSpecimenOntoHighBar1)));
        hang4thSpecimen1.setConstantHeadingInterpolation(Math.toRadians(0));

        hang4thSpecimen2 = new Path(new BezierLine(new Point(hang3rdSpecimenOntoHighBar1), new Point(hang4thSpecimenOntoHighBar2)));
        hang4thSpecimen2.setConstantHeadingInterpolation(Math.toRadians(0));

        hang4thSpecimen3 = new Path(new BezierCurve(new Point(hang4thSpecimenOntoHighBar2), hangingSpecimenControlPoint2, new Point(hang4thSpecimenOntoHighBar3)));
        hang4thSpecimen3.setConstantHeadingInterpolation(Math.toRadians(0));

        hang4thSpecimen4 = new Path(new BezierLine(new Point(hang4thSpecimenOntoHighBar3), new Point(hang4thSpecimenOntoHighBar4)));
        hang4thSpecimen4.setConstantHeadingInterpolation(Math.toRadians(0));


        /*** Hanging 5th Specimen Onto High Bar ***/
        hang5thSpecimen1 = new Path(new BezierCurve(new Point(hang4thSpecimenOntoHighBar3), hangingSpecimenControlPoint1, new Point(hang5thSpecimenOntoHighBar1)));
        hang5thSpecimen1.setConstantHeadingInterpolation(Math.toRadians(0));

        hang5thSpecimen2 = new Path(new BezierLine(new Point(hang5thSpecimenOntoHighBar1), new Point(hang5thSpecimenOntoHighBar2)));
        hang5thSpecimen2.setConstantHeadingInterpolation(Math.toRadians(0));

        hang5thSpecimen3 = new Path(new BezierLine(new Point(hang5thSpecimenOntoHighBar2), new Point(hang5thSpecimenOntoHighBar3)));
        hang5thSpecimen3.setConstantHeadingInterpolation(Math.toRadians(0));

    }

    public void runOpMode(){
        buildPaths();
        verticalArmSubSystem = new DougieArmSubSystem(hardwareMap);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        /** Building Autonomous Route **/
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowPath(follower, scorePreload1),
                        new FollowPath(follower, chained1stSamplePush),
                        new FollowPath(follower, chained2ndSamplePush),
                        new FollowPath(follower, chained3rdSamplePush)
                )
        );

        telemetry.addData("Status: ", "Ready to start!HJGHJVH");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            CommandScheduler.getInstance().run();
            follower.update();

            verticalArmSubSystem.UpdateVerticalSlidePIDFControl();
            verticalArmSubSystem.UpdateHorizontalSlidePIDFControl();
        }
    }
}