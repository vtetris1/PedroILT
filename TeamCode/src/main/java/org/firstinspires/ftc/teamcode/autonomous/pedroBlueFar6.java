/*package org.firstinspires.ftc.teamcode.autonomous;

import static java.lang.Thread.sleep;

import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.robotHardware;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class pedroBlueFar6 {

    @Autonomous(name = "Pedro auto bFar", group = "Autonomous")
    @Configurable // Panels
    public abstract class autonomousPedro extends OpMode {

        private TelemetryManager panelsTelemetry; // Panels Telemetry instance
        public Follower follower; // Pedro Pathing follower instance
        private int pathState; // Current autonomous path state (state machine)
        private Paths paths; // Paths defined in the Paths class
        private robotHardware robot = new robotHardware();


        public void init() {
            panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(55.05882352941176, 8.47058823529412, Math.toRadians(90)));

            paths = new Paths(follower); // Build paths

            panelsTelemetry.debug("Status", "Initialized");
            panelsTelemetry.update(telemetry);
        }

        public void runOpMode() {
            follower.update(); // Update Pedro Pathing
            //pathState = autonomousPathUpdate(); // Update autonomous state machine

            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);
        }

        private void stopAllMotors(){
            robot.motorintake.setPower(0.0);
            robot.stopShooter();
        }

        private void moveBeltUp(){
            robot.motorintake.setPower(1.0);
        }




        public class Paths {
            public PathChain Path1;
            public PathChain Path2;
            public PathChain Path3;
            public PathChain Path4;
            public PathChain Path5;

            public Paths(Follower follower) {
                Path4 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(56.000, 8.000),
                                        new Pose(56.000, 8.121),
                                        new Pose(56.000, 10.492),
                                        new Pose(56.000, 10.613),
                                        new Pose(56.000, 10.734),
                                        new Pose(56.000, 10.874),
                                        new Pose(56.000, 10.995),
                                        new Pose(56.000, 11.116),
                                        new Pose(56.000, 11.236),
                                        new Pose(56.000, 11.357),
                                        new Pose(56.000, 11.497),
                                        new Pose(56.000, 11.618),
                                        new Pose(56.000, 11.739),
                                        new Pose(56.000, 11.859),
                                        new Pose(56.000, 12.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(106))

                        .build();

                Path2 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(56.000, 12.000),
                                        new Pose(57.084, 34.741),
                                        new Pose(35.500, 35.000)
                                )
                        ).setTangentHeadingInterpolation()

                        .build();

                Path3 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(35.500, 35.000),

                                        new Pose(12.500, 35.000)
                                )
                        ).setTangentHeadingInterpolation()

                        .build();

                Path4 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(12.500, 35.000),
                                        new Pose(27.686, 8.608),
                                        new Pose(56.000, 14.000)
                                )
                        ).setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

                Path5 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(56.000, 14.000),

                                        new Pose(56.000, 12.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(106))

                        .build();
            }
        }



        public int autonomousPathUpdate() throws InterruptedException {
            // Add your state machine Here
            // Access paths with paths.pathName
            // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine

            switch (pathState){

                case 0:
                    follower.followPath(paths.Path1);
                    updateState(1);
                    break;

                case 1:
                    if(!follower.isBusy()) {
                        follower.followPath(paths.Path2,true);
                        // shoot all balls
                        robot.startShooterAtRPM(1840);

                        sleep(100); // let shooter spinup
                        moveBeltUp();
                        sleep(4000); // time to shoot
                        stopAllMotors();

                        updateState(2);
                        break;
                    }
                case 2:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path3);
                        updateState(3);
                        moveBeltUp(); //intaking
                        break;
                    }
                case 3:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path4);
                        updateState(4);
                        stopAllMotors();
                        break;
                    }
                case 4:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path5,true);
                        // shoot all balls
                        robot.startShooterAtRPM(1840);

                        sleep(100); // let shooter spinup
                        moveBeltUp();
                        sleep(4000); // time to shoot
                        stopAllMotors();

                        sleep(100); // ratelimit
                        updateState(2);
                        moveBeltUp(); //intaking
                        break;

                    }



                case 5:
                    if (!follower.isBusy()) {
                        if (!follower.isBusy()) {
                            follower.followPath(paths.Path5,true);
                            // shoot all balls


                        }
                    }

            }


            return pathState;
        }

        private void updateState(int newState){
            pathState = newState;

        }
    }

}


 */