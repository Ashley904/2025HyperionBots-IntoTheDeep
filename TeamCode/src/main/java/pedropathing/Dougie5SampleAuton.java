package pedropathing;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedropathing.constants.FConstants;
import pedropathing.constants.LConstants;

@Autonomous(name = "4 Sample Auto")
public class Dougie5SampleAuton extends LinearOpMode {

    DougieArmSubSystem armSubSystem;
    private Follower follower;


    // Starting Position of our robot
    private final Pose startPose = new Pose(10.5, 112, Math.toRadians(0));

    // Score preload
    private final Pose scoreSamplePreloadIntoHighBasket = new Pose(13.15, 127, Math.toRadians(-42));

    // Collect and score sample spike 1
    private final Pose collecting1stSampleSpike1 = new Pose(17.4, 122.85, Math.toRadians(0));
    private final Pose scoring1stSampleIntoHighBasket = new Pose(13.21, 127, Math.toRadians(-42));

    // Collect and score sample spike 2
    private final Pose collecting2ndSampleSpike2 = new Pose(16.5, 127.75, Math.toRadians(11.5));
    private final Pose scoring2ndSampleIntoHighBasket = new Pose(13.21, 127, Math.toRadians(-42));

    // Collect and score sample spike 3
    private final Pose collecting3rdSampleSpike3 = new Pose(26.5, 130, Math.toRadians(45.5));
    private final Pose scoring3rdSampleIntoHighBasket = new Pose(12, 127, Math.toRadians(-42));

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
        scoreSamplePreload.setZeroPowerAccelerationMultiplier(0.1);

        /*** Scoring 2nd Sample (Spike 1) ***/
        collect1stSample = new Path(new BezierLine(new Point(scoreSamplePreloadIntoHighBasket), new Point(collecting1stSampleSpike1)));
        collect1stSample.setLinearHeadingInterpolation(scoreSamplePreloadIntoHighBasket.getHeading(), collecting1stSampleSpike1.getHeading());
        collect1stSample.setZeroPowerAccelerationMultiplier(1);
        collect1stSample.setPathEndTimeoutConstraint(350);

        collect1stSampleChain = new PathChain(collect1stSample);

        scoring1stSample = new Path(new BezierLine(new Point(collecting1stSampleSpike1), new Point(scoring1stSampleIntoHighBasket)));
        scoring1stSample.setLinearHeadingInterpolation(Math.toRadians(0), scoring1stSampleIntoHighBasket.getHeading());
        scoring1stSample.setZeroPowerAccelerationMultiplier(0.1);

        /*** Scoring 3rd Sample (Spike 2) ***/
        collect2ndSample = new Path(new BezierLine(new Point(scoring1stSampleIntoHighBasket), new Point(collecting2ndSampleSpike2)));
        collect2ndSample.setLinearHeadingInterpolation(scoring1stSampleIntoHighBasket.getHeading(), collecting2ndSampleSpike2.getHeading());
        collect2ndSample.setZeroPowerAccelerationMultiplier(0.1);
        collect2ndSample.setPathEndTimeoutConstraint(350);

        collect2ndSampleChain = new PathChain(collect2ndSample);

        scoring2ndSample = new Path(new BezierLine(new Point(collecting2ndSampleSpike2), new Point(scoring2ndSampleIntoHighBasket)));
        scoring2ndSample.setLinearHeadingInterpolation(collecting2ndSampleSpike2.getHeading(), scoring2ndSampleIntoHighBasket.getHeading());

        /*** Scoring 4th Sample (Spike 2) ***/
        collect3rdSample = new Path(new BezierLine(new Point(scoring2ndSampleIntoHighBasket), new Point(collecting3rdSampleSpike3)));
        collect3rdSample.setLinearHeadingInterpolation(scoring2ndSampleIntoHighBasket.getHeading(), collecting3rdSampleSpike3.getHeading());
        collect3rdSample.setZeroPowerAccelerationMultiplier(0.1);
        collect3rdSample.setPathEndTimeoutConstraint(300);

        collect3rdSampleChain = new PathChain(collect3rdSample);

        scoring3rdSample = new Path(new BezierLine(new Point(collecting3rdSampleSpike3), new Point(scoring3rdSampleIntoHighBasket)));
        scoring3rdSample.setLinearHeadingInterpolation(collecting3rdSampleSpike3.getHeading(), scoring3rdSampleIntoHighBasket.getHeading());
    }

    public void runOpMode() {
        buildPaths();
        armSubSystem = new DougieArmSubSystem(hardwareMap);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();

        // Building Autonomous Route
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        /** Scoring sample preload **/

                        new InstantCommand(() -> armSubSystem.PositionForHighBucketScoring()),
                        new WaitCommand(1150),
                        new FollowPath(follower, scoreSamplePreload),
                        new WaitCommand(300),
                        new InstantCommand(() -> armSubSystem.ScoreSampleInHighBasket()),
                        new WaitCommand(650),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> armSubSystem.ScoreSampleInHighBasket()),
                                new InstantCommand(() -> armSubSystem.AutonomousPositionForSampleCollection(1525, 0.25))
                        ),


                        /** Collecting and Scoring spike 1 Sample **/
                        new FollowPath(follower, collect1stSampleChain, true),
                        new WaitCommand(1300),

                        new InstantCommand(() -> armSubSystem.SampleCollectionModeCollectSample()),
                        new WaitCommand((2500)),
                        new InstantCommand(() -> armSubSystem.TransferSampleToOuttake()),
                        new WaitCommand(1500),
                        new FollowPath(follower, scoring1stSample),

                        new WaitCommand(400),
                        new InstantCommand(() -> armSubSystem.ScoreSampleInHighBasket()),
                        new WaitCommand(800),


                        /** Collecting and Scoring spike 2 Sample **/
                        new ParallelCommandGroup(
                                new FollowPath(follower, collect2ndSampleChain, true),
                                new InstantCommand(() -> armSubSystem.AutonomousPositionForSampleCollection(1680, 0.25)),
                                new WaitCommand(1300)
                        ),

                        new InstantCommand(() -> armSubSystem.SampleCollectionModeCollectSample()),
                        new WaitCommand((2500)),
                        new InstantCommand(() -> armSubSystem.TransferSampleToOuttake()),
                        new WaitCommand(1270),
                        new FollowPath(follower, scoring1stSample),

                        new WaitCommand(400),
                        new InstantCommand(() -> armSubSystem.ScoreSampleInHighBasket()),
                        new WaitCommand(800),


                        /** Collecting and Scoring spike 3rd Sample **/
                        new ParallelCommandGroup(
                                new FollowPath(follower, collect3rdSampleChain, true),
                                new InstantCommand(() -> armSubSystem.AutonomousPositionForSampleCollection(1270, 0)),
                                new WaitCommand(1300)
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
}