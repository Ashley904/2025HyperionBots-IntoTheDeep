package pedropathing;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
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
import com.qualcomm.robotcore.hardware.Servo;

import pedropathing.constants.FConstants;
import pedropathing.constants.LConstants;

@Autonomous(name = "5 Specimen Auto + Park")
public class Dougie5SpecPreloadAuton extends LinearOpMode {

    Servo gripperServo;

    DougieArmSubSystem armSubSystem;

    private Follower follower;

    /**
     * Starting Position of our robot
     */
    private final Pose startPose = new Pose(10.75, 62.1, Math.toRadians(0));

    /**
     * Scoring the 1st specimen onto the high bar
     */
    private final Pose scoreSpecimenPreload = new Pose(41.53, 75, Math.toRadians(0));

    /**
     * Pushing the 1st sample into the observation zone
     */
    private final Pose push1stSampleIntoObservationZone1 = new Pose(46, 35, Math.toRadians(0));
    private final Pose push1stSampleIntoObservationZone2 = new Pose(57, 26, Math.toRadians(0));
    private final Pose push1stSampleIntoObservationZone3 = new Pose(31, 26, Math.toRadians(0));

    /**
     * Pushing the 2nd sample into the observation zone
     */
    private final Pose push2ndSampleIntoObservationZone1 = new Pose(46.5, 26.5, Math.toRadians(0));
    private final Pose push2ndSampleIntoObservationZone2 = new Pose(60, 15, Math.toRadians(0));
    private final Pose push2ndSampleIntoObservationZone3 = new Pose(27, 15, Math.toRadians(0));

    /**
     * Pushing the 3rd sample into the observation zone
     */
    private final Pose push3rdSampleIntoObservationZone1 = new Pose(50, 17.5, Math.toRadians(0));
    private final Pose push3rdSampleIntoObservationZone2 = new Pose(60, 10.7, Math.toRadians(0));
    private final Pose push3rdSampleIntoObservationZone3 = new Pose(17, 10.7, Math.toRadians(0));
    private final Pose reverseToCollectSample = new Pose(15, 12.15, Math.toRadians(0));

    /**
     * Hanging the 2nd specimen onto the high bar
     */
    private final Pose hang2ndSpecimenOntoHighBar1 = new Pose(42, 68, Math.toRadians(0));
    private final Pose hang2ndSpecimenOntoHighBar2 = new Pose(42.7, 73, Math.toRadians(0));

    /**
     * Hanging the 3rd specimen onto the high bar
     */
    private final Pose collect3rdSpecimenFromWall = new Pose(20, 34, Math.toRadians(0));
    private final Pose collect3rdSpecimenFromWall2 = new Pose(14.5, 34, Math.toRadians(0));
    private final Pose hang3rdSpecimenOntoHighBar1 = new Pose(41.5, 68, Math.toRadians (0));
    private final Pose hang3rdSpecimenOntoHighBar2 = new Pose(42.5, 72, Math.toRadians(0));
    private final Pose hang3rdSpecimenOntoHighBar3 = new Pose(42.51, 71.5, Math.toRadians(0));

    /**
     * Hanging the 4th specimen onto the high bar
     */
    private final Pose collect4thSpecimenFromWall = new Pose(20, 34, Math.toRadians(0));
    private final Pose collect4thSpecimenFromWall2 = new Pose(14.56, 34, Math.toRadians(0));
    private final Pose han4thSpecimenOntoHighBar1 = new Pose(41.5, 68, Math.toRadians (0));
    private final Pose hang4thSpecimenOntoHighBar2 = new Pose(42.5, 72, Math.toRadians(0));
    private final Pose hang4thSpecimenOntoHighBar3 = new Pose(42.51, 71.5, Math.toRadians(0));

    /**
     * Hanging the 5th specimen onto the high bar
     */
    private final Pose collect5thSpecimenFromWall = new Pose(16.2, 34, Math.toRadians(0));
    private final Pose hang5thSpecimenOntoHighBar1 = new Pose(41.7, 68, Math.toRadians(0));

    /**
     * Parking inside of the observation zone
     */
    private final Pose parkInObservationZone = new Pose(13, 30, Math.toRadians(0));


    private Path scorePreload;
    private PathChain scorePreloadPathChain;

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
    private Path pickUp3rdSpecimen;
    private PathChain chained3rdSamplePush;

    private Path hang2ndSpecimen1;
    private Path hang2ndSpecimen2;
    private PathChain chainedHang2ndSpecimen;

    private Path collect3rdSpecimen1;
    private Path collect3rdSpecimen2;
    private Path hang3rdSpecimen1;
    private Path hang3rdSpecimen2;
    private Path hang3rdSpecimen3;
    private PathChain chainedHang3rdSpecimen;
    private PathChain collect3rdSpecimenChain;

    private Path collect4thSpecimen1;
    private Path collect4thSpecimen2;
    private Path hang4thSpecimen1;
    private Path hang4thSpecimen2;
    private Path hang4thSpecimen3;
    private PathChain chainedHang4thSpecimen;
    private PathChain collect4thSpecimenChain;

    private Path collect5thSpecimen;
    private Path hang5thSpecimen1;
    private PathChain chainedHang5thSpecimen;

    private Path park;


    public void buildPaths() {
        /*** Scoring 1st Specimen ***/
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scoreSpecimenPreload)));
        scorePreload.setConstantHeadingInterpolation(Math.toRadians(0));
        scorePreload.setZeroPowerAccelerationMultiplier(4);
        scorePreloadPathChain = new PathChain(scorePreload);



        /*** Pushing 1st Sample Into Observation Zone ***/
        Point controlPoint1 = new Point(15, 33);
        push1stSample1 = new Path(new BezierCurve(new Point(scoreSpecimenPreload), controlPoint1, new Point(push1stSampleIntoObservationZone1)));
        push1stSample1.setConstantHeadingInterpolation(Math.toRadians(0));
        push1stSample1.setZeroPowerAccelerationMultiplier(10);

        Point controlPoint2 = new Point(52, 35);
        push1stSample2 = new Path(new BezierCurve(new Point(push1stSampleIntoObservationZone1), controlPoint2, new Point(push1stSampleIntoObservationZone2)));
        push1stSample2.setConstantHeadingInterpolation(Math.toRadians(0));
        push1stSample2.setZeroPowerAccelerationMultiplier(10);

        push1stSample3 = new Path(new BezierCurve(new Point(push1stSampleIntoObservationZone2), new Point(push1stSampleIntoObservationZone3)));
        push1stSample3.setConstantHeadingInterpolation(Math.toRadians(0));
        push1stSample3.setPathEndTimeoutConstraint(0);
        push1stSample3.setZeroPowerAccelerationMultiplier(10);

        chained1stSamplePush = new PathChain(push1stSample1, push1stSample2, push1stSample3);



        /*** Pushing 2nd Sample Into Observation Zone ***/
        push2ndSample1 = new Path(new BezierLine(new Point(push1stSampleIntoObservationZone3), new Point(push2ndSampleIntoObservationZone1)));
        push2ndSample1.setConstantHeadingInterpolation(Math.toRadians(0));
        push2ndSample1.setPathEndTimeoutConstraint(0);
        push2ndSample1.setZeroPowerAccelerationMultiplier(10);

        Point controlPoint4 = new Point(52, 26);
        push2ndSample2 = new Path(new BezierCurve(new Point(push2ndSampleIntoObservationZone1), controlPoint4, new Point(push2ndSampleIntoObservationZone2)));
        push2ndSample2.setConstantHeadingInterpolation(Math.toRadians(0));

        push2ndSample3 = new Path(new BezierLine(new Point(push2ndSampleIntoObservationZone2), new Point(push2ndSampleIntoObservationZone3)));
        push2ndSample3.setConstantHeadingInterpolation(Math.toRadians(0));
        push2ndSample3.setPathEndTimeoutConstraint(5);
        push2ndSample3.setZeroPowerAccelerationMultiplier(10);

        chained2ndSamplePush = new PathChain(push2ndSample1, push2ndSample2, push2ndSample3);



        /*** Pushing 3rd Sample Into Observation Zone ***/
        push3rdSample1 = new Path(new BezierLine(new Point(push2ndSampleIntoObservationZone3), new Point(push3rdSampleIntoObservationZone1)));
        push3rdSample1.setConstantHeadingInterpolation(Math.toRadians(0));
        push2ndSample3.setZeroPowerAccelerationMultiplier(10);

        Point controlPoint5 = new Point(54.5, 18);
        push3rdSample2 = new Path(new BezierCurve(new Point(push3rdSampleIntoObservationZone1), controlPoint5, new Point(push3rdSampleIntoObservationZone2)));
        push3rdSample2.setConstantHeadingInterpolation(Math.toRadians(0));
        push3rdSample1.setConstantHeadingInterpolation(Math.toRadians(0));
        push3rdSample2.setZeroPowerAccelerationMultiplier(1);

        push3rdSample3 = new Path(new BezierLine(new Point(push3rdSampleIntoObservationZone2), new Point(push3rdSampleIntoObservationZone3)));
        push3rdSample3.setConstantHeadingInterpolation(Math.toRadians(0));
        push3rdSample3.setZeroPowerAccelerationMultiplier(6);
        chained3rdSamplePush = new PathChain(push3rdSample1, push3rdSample2, push3rdSample3);

        pickUp3rdSpecimen = new Path(new BezierLine(new Point(push3rdSampleIntoObservationZone3), new Point(reverseToCollectSample)));
        pickUp3rdSpecimen.setConstantHeadingInterpolation(Math.toRadians(0));
        pickUp3rdSpecimen.setZeroPowerAccelerationMultiplier(4);



        /*** Hanging 2nd Specimen Onto High Bar ***/
        Point hangControlPoint1 = new Point(12, 50);
        hang2ndSpecimen1 = new Path(new BezierCurve(new Point(reverseToCollectSample), hangControlPoint1, new Point(hang2ndSpecimenOntoHighBar1)));
        hang2ndSpecimen1.setConstantHeadingInterpolation(Math.toRadians(0));
        hang2ndSpecimen1.setZeroPowerAccelerationMultiplier(10);

        hang2ndSpecimen2 = new Path(new BezierLine(new Point(hang2ndSpecimenOntoHighBar1), new Point(hang2ndSpecimenOntoHighBar2)));
        hang2ndSpecimen2.setConstantHeadingInterpolation(Math.toRadians(0));

        chainedHang2ndSpecimen = new PathChain(hang2ndSpecimen1, hang2ndSpecimen2);



        /*** Hanging 3rd Specimen Onto High Bar ***/
        Point collectSpecimenControlPoint1 = new Point(20, 70);
        Point collectSpecimenControlPoint2 = new Point(45, 34);
        collect3rdSpecimen1 = new Path(new BezierCurve(new Point(hang2ndSpecimenOntoHighBar2), collectSpecimenControlPoint1, collectSpecimenControlPoint2, new Point(collect3rdSpecimenFromWall)));
        collect3rdSpecimen1.setConstantHeadingInterpolation(Math.toRadians(0));
        collect3rdSpecimen1.setZeroPowerAccelerationMultiplier(10);

        collect3rdSpecimen2 = new Path(new BezierLine(new Point(collect3rdSpecimenFromWall), new Point(collect3rdSpecimenFromWall2)));
        collect3rdSpecimen2.setConstantHeadingInterpolation(Math.toRadians(0));
        collect3rdSpecimen2.setZeroPowerAccelerationMultiplier(4);

        collect3rdSpecimenChain = new PathChain(collect3rdSpecimen1, collect3rdSpecimen2);


        Point hangingSpecimenControlPoint2 = new Point(12, 50);
        hang3rdSpecimen1 = new Path(new BezierCurve(new Point(collect3rdSpecimenFromWall2), hangingSpecimenControlPoint2, new Point(hang3rdSpecimenOntoHighBar1)));
        hang3rdSpecimen1.setConstantHeadingInterpolation(Math.toRadians(0));
        hang3rdSpecimen1.setZeroPowerAccelerationMultiplier(10);

        hang3rdSpecimen2 = new Path(new BezierLine(new Point(hang3rdSpecimenOntoHighBar1), new Point(hang3rdSpecimenOntoHighBar2)));
        hang3rdSpecimen2.setConstantHeadingInterpolation(Math.toRadians(0));

        hang3rdSpecimen3 = new Path(new BezierLine(new Point(hang3rdSpecimenOntoHighBar2), new Point(hang3rdSpecimenOntoHighBar3)));
        hang3rdSpecimen3.setConstantHeadingInterpolation(Math.toRadians(0));

        chainedHang3rdSpecimen = new PathChain(hang3rdSpecimen1, hang3rdSpecimen2, hang3rdSpecimen3);



        /*** Hanging 4th Specimen Onto High Bar **/
        Point collectSpecimenControlPoint3 = new Point(20, 70);
        Point collectSpecimenControlPoint4 = new Point(45, 34);
        collect4thSpecimen1 = new Path(new BezierCurve(new Point(hang2ndSpecimenOntoHighBar2), collectSpecimenControlPoint3, collectSpecimenControlPoint4, new Point(collect3rdSpecimenFromWall)));
        collect4thSpecimen1.setConstantHeadingInterpolation(Math.toRadians(0));
        collect4thSpecimen1.setZeroPowerAccelerationMultiplier(10);

        collect4thSpecimen2 = new Path(new BezierLine(new Point(collect3rdSpecimenFromWall), new Point(collect3rdSpecimenFromWall2)));
        collect4thSpecimen2.setConstantHeadingInterpolation(Math.toRadians(0));
        collect4thSpecimen2.setZeroPowerAccelerationMultiplier(4);

        collect4thSpecimenChain = new PathChain(collect3rdSpecimen1, collect3rdSpecimen2);


        Point hangingSpecimenControlPoint3 = new Point(12, 50);
        hang4thSpecimen1 = new Path(new BezierCurve(new Point(collect3rdSpecimenFromWall2), hangingSpecimenControlPoint3, new Point(hang3rdSpecimenOntoHighBar1)));
        hang4thSpecimen1.setConstantHeadingInterpolation(Math.toRadians(0));
        hang4thSpecimen1.setZeroPowerAccelerationMultiplier(10);

        hang4thSpecimen2 = new Path(new BezierLine(new Point(hang3rdSpecimenOntoHighBar1), new Point(hang3rdSpecimenOntoHighBar2)));
        hang4thSpecimen2.setConstantHeadingInterpolation(Math.toRadians(0));

        hang4thSpecimen3 = new Path(new BezierLine(new Point(hang3rdSpecimenOntoHighBar2), new Point(hang3rdSpecimenOntoHighBar3)));
        hang4thSpecimen3.setConstantHeadingInterpolation(Math.toRadians(0));

        chainedHang4thSpecimen = new PathChain(hang4thSpecimen1, hang4thSpecimen2, hang4thSpecimen3);


        /*** Hanging 5th Specimen Onto High Bar **/
        Point collectSpecimenControlPoint5 = new Point(20, 70);
        Point collectSpecimenControlPoint6 = new Point(45, 30);
        collect5thSpecimen = new Path (new BezierCurve(new Point(hang4thSpecimenOntoHighBar2), collectSpecimenControlPoint5, collectSpecimenControlPoint6, new Point(collect5thSpecimenFromWall)));
        collect5thSpecimen.setConstantHeadingInterpolation(Math.toRadians(0));
        collect5thSpecimen.setZeroPowerAccelerationMultiplier(10);

        Point hangingSpecimenControlPoint6 = new Point(12, 50);
        hang5thSpecimen1 = new Path(new BezierCurve(new Point(collect5thSpecimenFromWall), hangingSpecimenControlPoint6, new Point(hang5thSpecimenOntoHighBar1)));
        hang5thSpecimen1.setConstantHeadingInterpolation(Math.toRadians(0));
        hang5thSpecimen1.setZeroPowerAccelerationMultiplier(10);

        chainedHang5thSpecimen = new PathChain(hang5thSpecimen1);

        Point parkInObservationZoneControlPoint = new Point(20, 70);
        park = new Path(new BezierCurve(new Point(hang5thSpecimenOntoHighBar1), parkInObservationZoneControlPoint, new Point(parkInObservationZone)));
        park.setConstantHeadingInterpolation(Math.toRadians(0));
        park.setZeroPowerAccelerationMultiplier(10);
    }

    public void runOpMode(){
        gripperServo = hardwareMap.get(Servo.class, "verticalGripperServo");
        gripperServo.setDirection(Servo.Direction.FORWARD);

        armSubSystem = new DougieArmSubSystem(hardwareMap);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        gripperServo.setPosition(0.625);
        buildPaths();

        // Building Autonomous Route
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        /** Scoring 1st Specimen Preload **/
                        new InstantCommand(() -> armSubSystem.PositionForSpecimenScoring()),
                        new WaitCommand(600),
                        new FollowPath(follower, scorePreloadPathChain, true),
                        new WaitCommand(395),
                        new InstantCommand(() -> armSubSystem.ScoreSpecimen()),
                        new WaitCommand(450),


                        /** Pushing All 3 Samples Into Observation Zone **/
                        new ParallelCommandGroup(
                                new InstantCommand(() -> armSubSystem.PositionForSpecimenCollection()),
                                new FollowPath(follower, chained1stSamplePush)
                        ),

                        new FollowPath(follower, chained2ndSamplePush),
                        new FollowPath(follower, chained3rdSamplePush),
                        new FollowPath(follower, pickUp3rdSpecimen),
                        new InstantCommand(() -> armSubSystem.PositionForSpecimenScoring()),
                        new WaitCommand(650),


                        /** Collecting + Hanging 2nd Specimen **/
                        new FollowPath(follower, chainedHang2ndSpecimen, true),
                        new WaitCommand(350),

                        new InstantCommand(() -> armSubSystem.ScoreSpecimen()),
                        new WaitCommand(450),


                        /** Collecting + Hanging 3rd Specimen **/

                        new ParallelCommandGroup(
                                new InstantCommand(() -> armSubSystem.PositionForSpecimenCollection()),
                                new FollowPath(follower, collect3rdSpecimenChain)
                        ),

                        new InstantCommand(() -> armSubSystem.PositionForSpecimenScoring()),
                        new WaitCommand(600),

                        new FollowPath(follower, chainedHang3rdSpecimen, true),
                        new InstantCommand(() -> armSubSystem.ScoreSpecimen()),
                        new WaitCommand(465),



                        /** Collecting + Hanging 4th Specimen **/

                        new ParallelCommandGroup(
                                new InstantCommand(() -> armSubSystem.PositionForSpecimenCollection()),
                                new FollowPath(follower, collect4thSpecimenChain)
                        ),
                        new InstantCommand(() -> armSubSystem.PositionForSpecimenScoring()),
                        new WaitCommand(600),

                        new FollowPath(follower, chainedHang4thSpecimen, true),
                        new InstantCommand(() -> armSubSystem.ScoreSpecimen()),
                        new WaitCommand(465),

                        new FollowPath(follower, park)


                        /** Collecting + Hanging 5th Specimen

                        new ParallelCommandGroup(
                                new InstantCommand(() -> armSubSystem.PositionForSpecimenCollection()),
                                new FollowPath(follower, collect5thSpecimen)
                        ),
                        new InstantCommand(() -> armSubSystem.PositionForSpecimenScoring()),
                        new WaitCommand(430),

                        new FollowPath(follower, chainedHang5thSpecimen, true),
                        new InstantCommand(() -> armSubSystem.ScoreSpecimen()),
                        new WaitCommand(465),

                        new ParallelCommandGroup(
                                new FollowPath(follower, park),
                                new InstantCommand(() -> armSubSystem.PositionForSpecimenCollection())
                        )
                        */
                )
        );


        telemetry.addData("Status: ", "Ready to start...");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            follower.update();

            armSubSystem.IntakeOpModeIdlePosition();

            armSubSystem.VerticalPIDFSlideControl();
            armSubSystem.HorizontalPIDFSlideControl();
            armSubSystem.updateServos();

            CommandScheduler.getInstance().run();
        }

    }
}
