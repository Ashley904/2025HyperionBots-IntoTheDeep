package pedropathing;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedropathing.constants.FConstants;
import pedropathing.constants.LConstants;

@Autonomous(name = "4 Sample Auton")
public class Dougie4SampleAuton extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState;

    /** Starting Position of our robot */
    private final Pose startPose = new Pose(10.5, 112, Math.toRadians(0));


    /** Scoring the 1st sample into the high bucket */
    private final Pose scoreSamplePreload = new Pose(17, 122.5, Math.toRadians(0));

    /** Scoring the 2nd sample into the high bucket */
    private final Pose score2ndSampleIntoHighBasket1 = new Pose(30, 121, Math.toRadians(0));
    private final Pose score2ndSampleIntoHighBasket2 = new Pose(17, 122.5, Math.toRadians(-42));

    /** Scoring the 3rd sample into the high bucket */
    private final Pose score3rdSampleIntoHighBasket1 = new Pose(30, 128.5, Math.toRadians(0));
    private final Pose score3rdSampleIntoHighBasket2 = new Pose(17, 122.5, Math.toRadians(-42));

    /** Scoring the 4th sample into the high bucket */
    private final Pose score4thSampleIntoHighBasket1 = new Pose(39, 120, Math.toRadians(90));
    private final Pose score4thSampleIntoHighBasket2 = new Pose(39, 134.5, Math.toRadians(90));
    private final Pose score4thSampleIntoHighBasket3 = new Pose(17, 122.5, Math.toRadians(-42));

    /** Park By Pit */
    private final Pose parkByPit = new Pose(70, 100, Math.toRadians(90));

    private Path scorePreload;


    private Path score2ndSample1;
    private Path score2ndSample2;

    private Path score3rdSample1;
    private Path score3rdSample2;

    private Path score4thSample1;
    private Path score4thSample2;
    private Path score4thSample3;
    private PathChain score4thSampleChain;

    private Path park;

    public void buildPaths() {

        /*** Scoring 1st Sample ***/
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scoreSamplePreload)));
        scorePreload.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-42));

        /*** Scoring 2nd Sample ***/
        score2ndSample1 = new Path(new BezierLine(new Point(scoreSamplePreload), new Point(score2ndSampleIntoHighBasket1)));
        score2ndSample1.setLinearHeadingInterpolation(Math.toRadians(-42), Math.toRadians(0));

        score2ndSample2 = new Path(new BezierLine(new Point(score2ndSampleIntoHighBasket1), new Point(score2ndSampleIntoHighBasket2)));
        score2ndSample2.setLinearHeadingInterpolation(Math.toRadians(0), score2ndSampleIntoHighBasket2.getHeading());

        /*** Scoring 3rd Sample ***/
        score3rdSample1 = new Path(new BezierLine(new Point(score2ndSampleIntoHighBasket2), new Point(score3rdSampleIntoHighBasket1)));
        score3rdSample1.setLinearHeadingInterpolation(Math.toRadians(-42), Math.toRadians(0));

        score3rdSample2 = new Path(new BezierLine(new Point(score3rdSampleIntoHighBasket1), new Point(score3rdSampleIntoHighBasket2)));
        score3rdSample2.setLinearHeadingInterpolation(Math.toRadians(0), score2ndSampleIntoHighBasket2.getHeading());

        /*** Scoring 4th Sample ***/
        score4thSample1 = new Path(new BezierLine(new Point(score3rdSampleIntoHighBasket2), new Point(score4thSampleIntoHighBasket1)));
        score4thSample1.setLinearHeadingInterpolation(Math.toRadians(-42), score4thSampleIntoHighBasket1.getHeading());

        score4thSample2 = new Path(new BezierLine(new Point(score4thSampleIntoHighBasket1), new Point(score4thSampleIntoHighBasket2)));
        score4thSample2.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));

        score4thSampleChain = new PathChain(score4thSample1, score4thSample2);

        score4thSample3 = new Path(new BezierLine(new Point(score4thSampleIntoHighBasket2), new Point(score4thSampleIntoHighBasket3)));
        score4thSample3.setLinearHeadingInterpolation(Math.toRadians(0), score2ndSampleIntoHighBasket2.getHeading());

        /*** Park***/
        park = new Path(new BezierLine(new Point(score4thSampleIntoHighBasket3), new Point(parkByPit)));
        park.setLinearHeadingInterpolation(Math.toRadians(-42), Math.toRadians(parkByPit.getHeading()));
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start following the first path
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                // Wait for the first path to finish
                if (!follower.isBusy()) {

                    follower.followPath(scorePreload);
                    setPathState(2);
                }
                break;
            case 2:
                // Wait for the first path to finish
                if (!follower.isBusy()) {
                    // Start the first chained path
                    follower.followPath(score2ndSample1);
                    setPathState(3);
                }
                break;
            case 3:
                // Wait for the first path to finish
                if (!follower.isBusy()) {
                    // Start the first chained path
                    follower.followPath(score2ndSample2);
                    setPathState(4);
                }
                break;
            case 4:
                // Wait for the first path to finish
                if (!follower.isBusy()) {
                    // Start the first chained path
                    follower.followPath(score3rdSample1);
                    setPathState(5);
                }
                break;
            case 5:
                // Wait for the first path to finish
                if (!follower.isBusy()) {
                    // Start the first chained path
                    follower.followPath(score3rdSample2);
                    setPathState(6);
                }
                break;
            case 6:
                // Wait for the first path to finish
                if (!follower.isBusy()) {
                    // Start the first chained path
                    follower.followPath(score4thSampleChain);
                    setPathState(7);
                }
                break;
            case 7:
                // Wait for the first path to finish
                if (!follower.isBusy()) {
                    // Start the first chained path
                    follower.followPath(score4thSample3);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    follower.followPath(park);
                    setPathState(9);
                }
            case 9:
                // Wait for the second chained path to finish
                if (!follower.isBusy()) {
                    // Stop after completing all paths
                    setPathState(-1); // Set to an invalid state to stop further pathing
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        autonomousPathUpdate();

        CommandScheduler.getInstance().run();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();

        CommandScheduler.getInstance().run();;
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0); // Start with the first path
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}