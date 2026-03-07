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
@TeleOp(name = "turretTest")
public class turretTest extends LinearOpMode {
    robotHardware robot = new robotHardware();
    //yo


    private long nowMs() {
        return System.currentTimeMillis();
    }

    @Override
    public void runOpMode() {
        init();
        robot.init(hardwareMap);
        start();
        int ticks = 1332;
        int autoTicks = 0;
        int tickStep = 0;


        robot.motorturret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.motorturret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorturret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();



        while (opModeIsActive()) {
            if(gamepad1.right_bumper){
                robot.motorturret.setTargetPosition(ticks);
                robot.motorturret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorturret.setPower(0.9);

                telemetry.addData("projected ticks: ", ticks);
                telemetry.addData("tick change ", tickStep);
                telemetry.addData("current ticks ", robot.motorturret.getCurrentPosition());
                telemetry.update();
            }

            if(gamepad1.right_trigger > 0.5){
                robot.motorturret.setTargetPosition(autoTicks);
                robot.motorturret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorturret.setPower(0.9);

                telemetry.addData("projected ticks: ", ticks);
                telemetry.addData("tick change ", tickStep);
                telemetry.addData("current ticks ", robot.motorturret.getCurrentPosition());
                telemetry.update();
            }

            if (gamepad1.left_trigger > 0.5){
                ticks = ticks - tickStep;
                    sleep(500);
            }
            if (gamepad1.left_bumper){
                ticks = ticks + tickStep;
                sleep(500);
            }



            if (gamepad1.x) tickStep = 1;
            else if (gamepad1.a) tickStep = 5;
            else if (gamepad1.b) tickStep = 10;
            else if (gamepad1.y) tickStep = 50;

            idle();
            telemetry.addData("projected ticks: ", ticks);
            telemetry.addData("tick change ", tickStep);
            telemetry.addData("current ticks ", robot.motorturret.getCurrentPosition());
            telemetry.update();
            telemetry.update();
        }


    }

}
