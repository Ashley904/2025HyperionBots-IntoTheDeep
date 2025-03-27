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

@Autonomous(name = "5+1 Specimen Auto With Sample Drop ")
public class Dougie5Spec1SampleAuton extends LinearOpMode {

    DougieArmSubSystem armSubSystem;

    private Follower follower;

    /**
     * Starting Position of our robot
     */
    private final Pose startPose = new Pose(10.5, 60, Math.toRadians(0));

    /**
     * Scoring the 1st specimen onto the high bar
     */
    private final Pose scoreSpecimenPreload = new Pose(41, 77.5, Math.toRadians(0));

    /**
     * Pushing the 1st sample into the observation zone
     */
    private final Pose push1stSampleIntoObservationZone1 = new Pose(46, 36.5, Math.toRadians(0));
    private final Pose push1stSampleIntoObservationZone2 = new Pose(57, 26.5, Math.toRadians(0));
    private final Pose push1stSampleIntoObservationZone3 = new Pose(32.5, 26.5, Math.toRadians(0));

    /**
     * Pushing the 2nd sample into the observation zone
     */
    private final Pose push2ndSampleIntoObservationZone1 = new Pose(46.5, 26.5, Math.toRadians(0));
    private final Pose push2ndSampleIntoObservationZone2 = new Pose(60, 17.5, Math.toRadians(0));
    private final Pose push2ndSampleIntoObservationZone3 = new Pose(27, 17.5, Math.toRadians(0));

    /**
     * Pushing the 3rd sample into the observation zone
     */
    private final Pose push3rdSampleIntoObservationZone1 = new Pose(50, 17.5, Math.toRadians(0));
    private final Pose push3rdSampleIntoObservationZone2 = new Pose(60, 10, Math.toRadians(0));
    private final Pose push3rdSampleIntoObservationZone3 = new Pose(15, 10, Math.toRadians(0));

    /**
     * Hanging the 2nd specimen onto the high bar
     */
    private final Pose hang2ndSpecimenOntoHighBar1 = new Pose(43.5, 70.5, Math.toRadians(0));
    private final Pose hang2ndSpecimenOntoHighBar2 = new Pose(44, 75, Math.toRadians(0));

    /**
     * Hanging the 3rd specimen onto the high bar
     */
    private final Pose collect3rdSpecimenFromWall = new Pose(13, 34, Math.toRadians(0));
    private final Pose hang3rdSpecimenOntoHighBar1 = new Pose(43.5, 71, Math.toRadians(0));
    private final Pose hang3rdSpecimenOntoHighBar2 = new Pose(44, 72, Math.toRadians(0));

    /**
     * Hanging the 4th specimen onto the high bar
     */
    private final Pose collect4thSpecimenFromWall = new Pose(13, 34, Math.toRadians(0));
    private final Pose hang4thSpecimenOntoHighBar1 = new Pose(43.5, 71, Math.toRadians(0));
    private final Pose hang4thSpecimenOntoHighBar2 = new Pose(44, 76, Math.toRadians(0));

    /**
     * Hanging the 5th specimen onto the high bar
     */
    private final Pose collect5thSpecimenFromWall = new Pose(13, 34, Math.toRadians(0));
    private final Pose hang5thSpecimenOntoHighBar1 = new Pose(43.5, 71, Math.toRadians(0));
    private final Pose hang5thSpecimenOntoHighBar2 = new Pose(44, 71.5, Math.toRadians(0));

    /**
     * Collect and Score Sample Into High Bucket
     */
    private final Pose collectSampleFromWall = new Pose(13, 34, Math.toRadians(0));
    private final Pose scoreSampleIntoHighBasket = new Pose(15.5, 123.5, Math.toRadians(0));


    private Path scorePreload;

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
    private PathChain chainedHang2ndSpecimen;

    private Path collect3rdSpecimen;
    private Path hang3rdSpecimen1;
    private Path hang3rdSpecimen2;
    private PathChain chainedHang3rdSpecimen;

    private Path collect4thSpecimen;
    private Path hang4thSpecimen1;
    private Path hang4thSpecimen2;
    private PathChain chainedHang4thSpecimen;

    private Path collect5thSpecimen;
    private Path hang5thSpecimen1;
    private Path hang5thSpecimen2;
    private PathChain chainedHang5thSpecimen;

    private Path collectSample;
    private Path scoreSample;


    public void buildPaths() {
        /*** Scoring 1st Specimen ***/
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scoreSpecimenPreload)));
        scorePreload.setConstantHeadingInterpolation(Math.toRadians(0));



        /*** Pushing 1st Sample Into Observation Zone ***/
        Point controlPoint1 = new Point(15, 40);
        push1stSample1 = new Path(new BezierCurve(new Point(scoreSpecimenPreload), controlPoint1, new Point(push1stSampleIntoObservationZone1)));
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

        Point controlPoint4 = new Point(56.5, 24);
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

        Point controlPoint5 = new Point(57, 16.5);
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

        chainedHang2ndSpecimen = new PathChain(hang2ndSpecimen1, hang2ndSpecimen2);



        /*** Hanging 3rd Specimen Onto High Bar ***/
        Point collectSpecimenControlPoint1 = new Point(20, 70);
        Point collectSpecimenControlPoint2 = new Point(45, 30);
        collect3rdSpecimen = new Path(new BezierCurve(new Point(hang2ndSpecimenOntoHighBar2), collectSpecimenControlPoint1, collectSpecimenControlPoint2, new Point(collect3rdSpecimenFromWall)));
        collect3rdSpecimen.setConstantHeadingInterpolation(Math.toRadians(0));


        Point hangingSpecimenControlPoint2 = new Point(12, 50);
        hang3rdSpecimen1 = new Path(new BezierCurve(new Point(collect3rdSpecimenFromWall), hangingSpecimenControlPoint2, new Point(hang3rdSpecimenOntoHighBar1)));
        hang3rdSpecimen1.setConstantHeadingInterpolation(Math.toRadians(0));

        hang3rdSpecimen2 = new Path(new BezierLine(new Point(hang3rdSpecimenOntoHighBar1), new Point(hang3rdSpecimenOntoHighBar2)));
        hang3rdSpecimen2.setConstantHeadingInterpolation(Math.toRadians(0));

        chainedHang3rdSpecimen = new PathChain(hang3rdSpecimen1, hang3rdSpecimen2);



        /*** Hanging 4th Specimen Onto High Bar **/
        Point collectSpecimenControlPoint3 = new Point(20, 70);
        Point collectSpecimenControlPoint4 = new Point(45, 30);
        collect4thSpecimen = new Path(new BezierCurve(new Point(hang3rdSpecimenOntoHighBar2), collectSpecimenControlPoint3, collectSpecimenControlPoint4, new Point(collect4thSpecimenFromWall)));
        collect4thSpecimen.setConstantHeadingInterpolation(Math.toRadians(0));


        Point hangingSpecimenControlPoint5 = new Point(12, 50);
        hang4thSpecimen1 = new Path(new BezierCurve(new Point(collect4thSpecimenFromWall), hangingSpecimenControlPoint5, new Point(hang4thSpecimenOntoHighBar1)));
        hang4thSpecimen1.setConstantHeadingInterpolation(Math.toRadians(0));

        hang4thSpecimen2 = new Path(new BezierLine(new Point(hang4thSpecimenOntoHighBar1), new Point(hang4thSpecimenOntoHighBar2)));
        hang4thSpecimen2.setConstantHeadingInterpolation(Math.toRadians(0));

        chainedHang4thSpecimen = new PathChain(hang4thSpecimen1, hang4thSpecimen2);


        /*** Hanging 5th Specimen Onto High Bar **/
        Point collectSpecimenControlPoint5 = new Point(20, 70);
        Point collectSpecimenControlPoint6 = new Point(45, 30);
        collect5thSpecimen = new Path(new BezierCurve(new Point(hang4thSpecimenOntoHighBar2), collectSpecimenControlPoint5, collectSpecimenControlPoint6, new Point(collect5thSpecimenFromWall)));
        collect5thSpecimen.setConstantHeadingInterpolation(Math.toRadians(0));

        Point hangingSpecimenControlPoint6 = new Point(12, 50);
        hang5thSpecimen1 = new Path(new BezierCurve(new Point(collect5thSpecimenFromWall), hangingSpecimenControlPoint6, new Point(hang5thSpecimenOntoHighBar1)));
        hang5thSpecimen1.setConstantHeadingInterpolation(Math.toRadians(0));

        hang5thSpecimen2 = new Path(new BezierLine(new Point(hang5thSpecimenOntoHighBar1), new Point(hang5thSpecimenOntoHighBar2)));
        hang5thSpecimen2.setConstantHeadingInterpolation(Math.toRadians(0));

        chainedHang5thSpecimen = new PathChain(hang5thSpecimen1, hang5thSpecimen2);

        /*** Parking In Observation Zone **/
        Point collectSampleControlPoint1 = new Point(20, 70);
        Point collectSampleControlPoint2 = new Point(45, 30);
        collectSample = new Path(new BezierCurve(new Point(hang5thSpecimenOntoHighBar2), collectSampleControlPoint1, collectSampleControlPoint2, new Point(collectSampleFromWall)));
        collectSample.setConstantHeadingInterpolation(Math.toRadians(0));
        collectSample.setZeroPowerAccelerationMultiplier(6.5);
        collectSample.setPathEndTimeoutConstraint(0);

        scoreSample = new Path(new BezierLine(new Point(collectSampleFromWall), new Point(scoreSampleIntoHighBasket)));
        scoreSample.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-42));
        scoreSample.setZeroPowerAccelerationMultiplier(6.5);
        scoreSample.setPathEndTimeoutConstraint(0);

    }

    public void runOpMode(){

        buildPaths();
        armSubSystem = new DougieArmSubSystem(hardwareMap);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        // Building Autonomous Route
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        /** Scoring 1st Specimen Preload **/
                        new InstantCommand(() -> armSubSystem.PositionForSpecimenScoring()),
                        new WaitCommand(300),
                        new FollowPath(follower, scorePreload),
                        new InstantCommand(() -> armSubSystem.ScoreSpecimen()),
                        new WaitCommand(450),


                        /** Pushing All 3 Samples Into Observation Zone **/
                        new ParallelCommandGroup(
                                new InstantCommand(() -> armSubSystem.PositionForSpecimenCollection()),
                                new FollowPath(follower, chained1stSamplePush)
                        ),
                        new FollowPath(follower, chained2ndSamplePush),
                        new FollowPath(follower, chained3rdSamplePush),

                        /** Collecting + Hanging 2nd Specimen **/
                        new InstantCommand(() -> armSubSystem.PositionForSpecimenScoring()),
                        new WaitCommand(300),
                        new FollowPath(follower, chainedHang2ndSpecimen),

                        new InstantCommand(() -> armSubSystem.ScoreSpecimen()),
                        new WaitCommand(450),

                        /** Collecting + Hanging 3rd Specimen **/

                        new ParallelCommandGroup(
                                new InstantCommand(() -> armSubSystem.PositionForSpecimenCollection()),
                                new FollowPath(follower, collect3rdSpecimen)
                        ),
                        new InstantCommand(() -> armSubSystem.PositionForSpecimenScoring()),
                        new WaitCommand(300),

                        new FollowPath(follower, chainedHang3rdSpecimen),
                        new InstantCommand(() -> armSubSystem.ScoreSpecimen()),
                        new WaitCommand(450),



                        /** Collecting + Hanging 4th Specimen **/

                        new ParallelCommandGroup(
                                new InstantCommand(() -> armSubSystem.PositionForSpecimenCollection()),
                                new FollowPath(follower, collect4thSpecimen)
                        ),
                        new InstantCommand(() -> armSubSystem.PositionForSpecimenScoring()),
                        new WaitCommand(300),

                        new FollowPath(follower, chainedHang4thSpecimen),
                        new InstantCommand(() -> armSubSystem.ScoreSpecimen()),
                        new WaitCommand(450),




                        /** Collecting + Hanging 5th Specimen **/

                        new ParallelCommandGroup(
                                new InstantCommand(() -> armSubSystem.PositionForSpecimenCollection()),
                                new FollowPath(follower, collect5thSpecimen)
                        ),
                        new InstantCommand(() -> armSubSystem.PositionForSpecimenScoring()),
                        new WaitCommand(300),

                        new FollowPath(follower, chainedHang5thSpecimen),
                        new InstantCommand(() -> armSubSystem.ScoreSpecimen()),
                        new WaitCommand(450),



                        /** Collect And Drop Of Sample Into High Basket**/

                        new ParallelCommandGroup(
                                new InstantCommand(() -> armSubSystem.PositionForSpecimenCollection()),
                                new FollowPath(follower, collectSample)
                        ),
                        new InstantCommand(() -> armSubSystem.PositionForSampleHighBasketScoring()),
                        new WaitCommand(300),

                        new FollowPath(follower, scoreSample),

                        new InstantCommand(() -> armSubSystem.DropSampleIntoHighBucket())

                )
        );

        telemetry.addData("Status: ", "Ready to start...");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            CommandScheduler.getInstance().run();
            follower.update();

            armSubSystem.VerticalPIDFSlideControl();
        }

    }
}