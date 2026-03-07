package org.firstinspires.ftc.teamcode.autonomous; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Example PedroPathing Auto Full Blue", group = "Examples")
public class pedroBlueFar9 extends OpMode {
//teshk
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    //private final Pose startPose = new Pose(28.5, 128, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(57, 8.5, Math.toRadians(180));
    private final Pose humanPickup1Pose = new Pose(10, 9, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose humanPickup2Pose = new Pose(10.5, 9, Math.toRadians(190));// Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose firstRowPickup1Pose = new Pose(42, 36, Math.toRadians(180));// Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose firstRowControl1Pose = new Pose(57.67, 32.66, Math.toRadians(180));
    private final Pose firstRowPickup2Pose = new Pose(12, 36, Math.toRadians(180));// Middle (Second Set) of Artifacts from the Spike Mark.// Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose firstRowControl2Pose = new Pose(19.1, 13.2, Math.toRadians(180));
    private final Pose finalParkingPose = new Pose(10.75, 8.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.

    private Path scorePreload;
    private PathChain humanPickup1, humanPickup2, humanScore;
    private PathChain firstPickup1, firstRowPickup2, firstRowScore;

    private PathChain finalParking;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        //scorePreload = new Path(new BezierLine(startPose, scorePose));
        //scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        // Score position to first row samples. We are using a single path with a BezierLine, which is a straight line.
        humanPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, humanPickup1Pose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        // First row samples pickup. We are using a single path with a BezierLine, which is a straight line.
        humanPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(humanPickup1Pose, humanPickup2Pose))
                .setLinearHeadingInterpolation(humanPickup1Pose.getHeading(), humanPickup1Pose.getHeading())
                .build();

        // After pickup go back to score pos. We are using a single path with a BezierLine, which is a straight line.
        humanScore = follower.pathBuilder()
                .addPath(new BezierLine(humanPickup2Pose, scorePose))
                .setLinearHeadingInterpolation(humanPickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        // Score position to first row samples. We are using a single path with a BezierLine, which is a straight line.
        firstPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, firstRowControl1Pose, firstRowPickup1Pose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        // First row samples pickup. We are using a single path with a BezierLine, which is a straight line.
        firstRowPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(firstRowPickup1Pose, firstRowPickup2Pose))
                .setConstantHeadingInterpolation(firstRowPickup1Pose.getHeading())
                .build();

        // After pickup go back to score pos. We are using a single path with a BezierLine, which is a straight line.
        firstRowScore = follower.pathBuilder()
                .addPath(new BezierCurve(firstRowPickup2Pose, firstRowControl2Pose, firstRowPickup2Pose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        finalParking = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, finalParkingPose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        /*

        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"


                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position
                if (!follower.isBusy()) {
                    /* Score Preload
                    //sleep(2000);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample
                    //if(pathTimer.getElapsedTimeSeconds() > 3) {
                    if (doneShooting()) {
                        follower.followPath(firstRowPickup1, true);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        follower.followPath(firstRowPickup2, true);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample
                    if (doneCollecting()) {
                        follower.followPath(firstRow2Score, true);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample
                    if (doneShooting()) {
                        follower.followPath(secondRowPickup1, true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample
                    if (pathTimer.getElapsedTimeSeconds() > 2) {
                        follower.followPath(secondRowPickup2, true);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample
                    if (doneCollecting()) {
                        follower.followPath(secondRow2Score, true);
                        setPathState(7);
                    }
                }
                break;
        }
        */
    }


    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public boolean doneShooting() {
        if(pathTimer.getElapsedTimeSeconds() > 2) {
            pathTimer.resetTimer();
            return true;
        }
        else {
            return false;
        }
    }

    public boolean doneCollecting() {
        if(pathTimer.getElapsedTimeSeconds() > 2) {
            pathTimer.resetTimer();
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(scorePose);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}