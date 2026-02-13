package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.robotHardware;
import java.util.function.Supplier;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.hardware.robotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Configurable
@TeleOp(name = "Opmode2")
public class Opmode2 extends LinearOpMode {

    robotHardware robot = new robotHardware();

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevY = false;
    private boolean prevX = false;

    private boolean parkMode = false;

    private final double SHOOTER_RPM_SHORT = 1400.0; // 28x2786/60 //28
    private final double SHOOTER_RPM_LONG = 1840; //36?
    private final double SHOOTER_RPM_CLEAR = -1000;
    private final double SHOOTER_CHANGE = 100;
    private double rpm = SHOOTER_RPM_SHORT;



    private double controller1Speed = 1;

    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;


    private long nowMs() {
        return System.currentTimeMillis();
    }


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            loop();

            boolean a = gamepad2.a;
            boolean b = gamepad2.b;
            boolean y = gamepad2.y;
            boolean x = gamepad2.x;

//DRIVING
            double x_dir = gamepad1.left_stick_x * 1.3 * controller1Speed;
            double y_dir = -gamepad1.left_stick_y * 1.3 * controller1Speed;
            double turn = gamepad1.right_stick_x * 1.3 * controller1Speed;

            double flPower = x_dir + y_dir + turn;
            double blPower = y_dir - x_dir + turn;
            double frPower = y_dir - x_dir - turn;
            double brPower = y_dir + x_dir - turn;

            robot.setDrivePower(flPower, frPower, blPower, brPower);

            if (gamepad1.dpad_down) {
                controller1Speed = 0.2;
            } else {
                controller1Speed = 1;
            }

            //intake/middle

            if (gamepad1.left_bumper){
                robot.motorintake.setPower(-1.0);
            } else if (gamepad1.left_trigger > 0.5){
                robot.motorintake.setPower(1.0);
            } else {
                robot.motorintake.setPower(0);
            }

            if (gamepad1.right_bumper){
                robot.servoL.setPower(-1);
                robot.servoR.setPower(1);
            } else if (gamepad1.right_trigger > 0.5){
                robot.servoL.setPower(1);
                robot.servoR.setPower(-1);
            } else {
                robot.servoL.setPower(0);
                robot.servoR.setPower(0);
            }

            //shooting

            if(b && !prevB){
                rpm += SHOOTER_CHANGE;
                sleep(50);
            } if(x && !prevX){
                rpm -= SHOOTER_CHANGE;
                sleep(50);
            }
            /*
            TO-DO: (jimmy if u see this please help)
            make a mode for shooting short vs shooting long
            debug shooting part
            turret programming
            check if telemetry/rpm switching works

             */
            //this doesnt work
            if (y && !prevY){
                robot.autoShoot(rpm);
                //find a solution for this
            }
            if (gamepad2.right_bumper){
                robot.autoShoot(SHOOTER_RPM_SHORT);
            }

            if (gamepad2.right_trigger > 0.5){
                robot.autoShoot(SHOOTER_RPM_LONG);
            }

            if(a && !prevA){
                robot.stopShooter();
            }

            prevA = a;
            prevB = b;
            prevY = y;
            prevX = x;



            //turret

            if (gamepad2.left_bumper){
                robot.motorturret.setPower(0.7);
            } else if (gamepad2.left_trigger > 0.8){
                robot.motorturret.setPower(-0.7);
            } else {
                robot.motorturret.setPower(0);
            }





            //elevator
            if(gamepad1.dpad_down && gamepad2.dpad_left && !parkMode){
                //robot.elevator.(); negative ticks
                //2786.2 ticks per rotation
                robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.elevator.setTargetPosition(-33434);
                robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.elevator.setPower(0.7);

                parkMode = true;
            }



            idle();
            // --- TELEMETRY ---
            telemetry.addData("intakePower", robot.motorintake.getPower());
            telemetry.addData("shooter velocity", robot.motorshoot.getVelocity());
            telemetry.addData("shooter tpr", robot.motorshoot.getMotorType().getTicksPerRev());
            telemetry.addData("projected rpm", rpm);
            telemetry.update();

        }


    }





}








