package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.hardware.robotHardware;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "motorpidf")
public class pidfTest extends LinearOpMode {

    robotHardware robot = new robotHardware();
    //yo
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevY = false;
    private boolean prevX = false;

    private final double SHOOTER_RPM_SHORT = 1455.0; // 28x2786/60 //28
    private final double SHOOTER_RPM_LONG = 1840; //36?
    private final double SHOOTER_RPM_CLEAR = -1000;

    private double controller1Speed = 1;

    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.3;



    private long nowMs() {
        return System.currentTimeMillis();
    }

    @Override
    public void runOpMode() {
        init();
        robot.init(hardwareMap);
        start();
        double rpm = 1400;
        double kp = 305.38;
        double ki = 0.15;
        double kd = 0.01;
        double kf = 11.5;

        int selectedState = 0;
        double pidStep = 0.0;


        robot.motorshoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.motorshoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();



        while (opModeIsActive()) {
            if(gamepad1.right_bumper){
                robot.motorshoot.setVelocity(rpm);
                telemetry.addData("kp", kp);
                telemetry.addData("ki", ki);
                telemetry.addData("kd", kd);
                telemetry.addData("kf", kf);
                telemetry.addData("pidStep: ", pidStep);
                telemetry.addData("selectedState: ", selectedState);
                telemetry.addData("rpm: ", robot.motorshoot.getVelocity());
                telemetry.update();
            }
            if(gamepad1.right_trigger > 0.5){
                robot.motorshoot.setVelocity(rpm / 2);
            }



            if (gamepad1.dpad_up) {
                selectedState = 0;
                sleep(500);
            }

            else if(gamepad1.dpad_left){
                selectedState = 1;
                sleep(500);
            }

            else if (gamepad1.dpad_down) {
                selectedState = 2;
                sleep(500);
            }

            else if (gamepad1.dpad_right){
                selectedState = 3;
                sleep(500);
            }



            if (gamepad1.left_trigger > 0.5){
                if (selectedState == 0){
                    kp -= pidStep;
                    sleep(500);
                }
                else if (selectedState == 1){
                    ki -= pidStep;
                    sleep(500);
                }
                else if (selectedState == 2){
                    kd -= pidStep;
                    sleep(500);
                }

                else if (selectedState == 3){
                    kf -= pidStep;
                    sleep(500);
                }

            }
            else if (gamepad1.left_bumper){
                if (selectedState == 0){
                    kp += pidStep;
                    sleep(500);
                }
                else if (selectedState == 1){
                    ki += pidStep;
                    sleep(500);
                }
                else if (selectedState == 2){
                    kd += pidStep;
                    sleep(500);
                }

                else if (selectedState == 3){
                    kf -= pidStep;
                    sleep(500);
                }
            }


            if (gamepad1.x) pidStep = 0.12;
            else if (gamepad1.a) pidStep = 1.0;
            else if (gamepad1.b) pidStep = 10.0;
            else if (gamepad1.y) pidStep = 100.0;

            robot.motorshoot.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(kp, ki, kd, kf)
            );

            idle();
            telemetry.addData("kp", kp);
            telemetry.addData("ki", ki);
            telemetry.addData("kd", kd);
            telemetry.addData("kf", kf);
            telemetry.addData("pidStep: ", pidStep);
            telemetry.addData("selectedState: ", selectedState);
            telemetry.addData("rpm: ", robot.motorshoot.getVelocity());
            telemetry.update();
        }


    }

}
