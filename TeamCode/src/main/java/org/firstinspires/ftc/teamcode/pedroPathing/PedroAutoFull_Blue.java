
package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Example PedroPathing Auto Full Blue", group = "Examples")
public class PedroAutoFull_Blue extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    //private final Pose startPose = new Pose(28.5, 128, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose startPose = new Pose(20, 122, Math.toRadians(140)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(48, 82, Math.toRadians(136)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose firstRowPickup1Pose = new Pose(50, 78, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose firstRowPickup2Pose = new Pose(25, 78, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose secondRowPickup1Pose = new Pose(50, 54, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose secondRowPickup2Pose = new Pose(25, 54, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose thirdRowPickup1Pose = new Pose(50, 30, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose thirdRowPickup2Pose = new Pose(25, 30, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.

    private final Pose finalParkingPose = new Pose(41, 38, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.

    private Path scorePreload;
    private PathChain firstRowPickup1, firstRowPickup2, firstRow2Score;
    private PathChain secondRowPickup1, secondRowPickup2, secondRow2Score;
    private PathChain thirdRowPickup1, thirdRowPickup2, thirdRow2Score;

    private PathChain finalParking;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        // Score position to first row samples. We are using a single path with a BezierLine, which is a straight line.
        firstRowPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, firstRowPickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), firstRowPickup1Pose.getHeading())
                .build();

        // First row samples pickup. We are using a single path with a BezierLine, which is a straight line.
        firstRowPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(firstRowPickup1Pose, firstRowPickup2Pose))
                .setLinearHeadingInterpolation(firstRowPickup1Pose.getHeading(), firstRowPickup2Pose.getHeading())
                .build();

        // After pickup go back to score pos. We are using a single path with a BezierLine, which is a straight line.
        firstRow2Score = follower.pathBuilder()
                .addPath(new BezierLine(firstRowPickup2Pose, scorePose))
                .setLinearHeadingInterpolation(firstRowPickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        // Score position to first row samples. We are using a single path with a BezierLine, which is a straight line.
        secondRowPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, secondRowPickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondRowPickup1Pose.getHeading())
                .build();

        // First row samples pickup. We are using a single path with a BezierLine, which is a straight line.
        secondRowPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(secondRowPickup1Pose, secondRowPickup2Pose))
                .setLinearHeadingInterpolation(secondRowPickup1Pose.getHeading(), secondRowPickup2Pose.getHeading())
                .build();

        // After pickup go back to score pos. We are using a single path with a BezierLine, which is a straight line.
        secondRow2Score = follower.pathBuilder()
                .addPath(new BezierLine(secondRowPickup2Pose, scorePose))
                .setLinearHeadingInterpolation(secondRowPickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        // Score position to first row samples. We are using a single path with a BezierLine, which is a straight line.
        thirdRowPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, thirdRowPickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), thirdRowPickup1Pose.getHeading())
                .build();

        // First row samples pickup. We are using a single path with a BezierLine, which is a straight line.
        thirdRowPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(thirdRowPickup1Pose, thirdRowPickup2Pose))
                .setLinearHeadingInterpolation(thirdRowPickup1Pose.getHeading(), thirdRowPickup2Pose.getHeading())
                .build();

        // After pickup go back to score pos. We are using a single path with a BezierLine, which is a straight line.
        thirdRow2Score = follower.pathBuilder()
                .addPath(new BezierLine(thirdRowPickup2Pose, scorePose))
                .setLinearHeadingInterpolation(thirdRowPickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        finalParking = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, finalParkingPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), finalParkingPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
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
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    //sleep(2000);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    //if(pathTimer.getElapsedTimeSeconds() > 3) {
                    if(doneShooting()) {
                        follower.followPath(firstRowPickup1, true);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if(pathTimer.getElapsedTimeSeconds() > 1) {
                        follower.followPath(firstRowPickup2, true);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(doneCollecting()) {
                        follower.followPath(firstRow2Score, true);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if(doneShooting()) {
                        follower.followPath(secondRowPickup1, true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(pathTimer.getElapsedTimeSeconds() > 2) {
                        follower.followPath(secondRowPickup2, true);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if(doneCollecting()) {
                        follower.followPath(secondRow2Score, true);
                        setPathState(7);
                    }
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    if(doneShooting()) {
                        follower.followPath(thirdRowPickup1, true);
                        setPathState(8);
                    }
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    if(pathTimer.getElapsedTimeSeconds() > 2) {
                        follower.followPath(thirdRowPickup2, true);
                        setPathState(9);
                    }
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    if(doneCollecting()) {
                        follower.followPath(thirdRow2Score, true);
                        setPathState(10);
                    }
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    // do shooting here
                    if (doneShooting()) {
                        follower.followPath(finalParking, true);
                        setPathState(11);
                    }
                }
                break;

            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    // do shooting here
                        setPathState(12);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
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
        follower.setStartingPose(startPose);

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