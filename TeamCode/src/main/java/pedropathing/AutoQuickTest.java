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

import org.opencv.core.Mat;

import pedropathing.constants.FConstants;
import pedropathing.constants.LConstants;

@Autonomous(name="Auto test")
public class AutoQuickTest extends LinearOpMode{

    private Follower follower;


    /**
     * Starting Position of our robot
     */
    private final Pose startPose = new Pose(10.5, 60, Math.toRadians(0));

    /**
     * Scoring the 1st specimen onto the high bar
     */
    private final Pose scoreSpecimenPreload = new Pose(12, 65, Math.toRadians(0));

    private Path scorePreload;

    void buildPaths(){

        /*** Scoring 1st Specimen ***/
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scoreSpecimenPreload)));
        scorePreload.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));

    }

    public void runOpMode(){

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();

        CommandScheduler.getInstance().schedule(

                new SequentialCommandGroup(
                        /** Scoring 1st Specimen Preload **/
                        new FollowPath(follower, scorePreload)
                )
        );

        telemetry.addData("Status: ", "Ready to start...");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            CommandScheduler.getInstance().run();
            follower.update();

        }

    }


}
