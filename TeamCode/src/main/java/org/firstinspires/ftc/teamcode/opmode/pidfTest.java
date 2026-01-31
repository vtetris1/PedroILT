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
@TeleOp(name = "pidfTest")
public class pidfTest extends LinearOpMode {

    robotHardware robot = new robotHardware();


    private long nowMs() {
        return System.currentTimeMillis();
    }

    @Override
    public void runOpMode() {
        init();
        robot.init(hardwareMap);
        start();
        double rpm = 1000.0;
        double kp = 15;
        double ki = 0.1;
        double kd = 2.3;

        int selectedState = 0;
        double pidStep = 0.0;


        robot.motorshoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.motorshoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorshoot.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kp, ki, kd, 0));
        waitForStart();



        while (opModeIsActive()) {
            loop();

            if(gamepad1.a){
                robot.motorshoot.setVelocity(rpm);
            }

            if (gamepad1.dpad_up && selectedState >= 2) {
                selectedState = 0;
            }
            else if (gamepad1.dpad_up){
                selectedState++;
            }

            if (gamepad1.dpad_down && selectedState <= 0) {
                selectedState = 2;
            }
            else if (gamepad1.dpad_down){
                selectedState--;
            }


            if (gamepad1.x) pidStep = 0.1;


        }


    }

}


