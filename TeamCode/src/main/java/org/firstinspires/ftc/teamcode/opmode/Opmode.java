package org.firstinspires.ftc.teamcode.opmode;

import org.firstinspires.ftc.teamcode.hardware.robotHardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import java.util.function.Supplier;

@Configurable
@Disabled
@TeleOp(name = "Opmode")
public class Opmode extends LinearOpMode {

    robotHardware robot = new robotHardware();

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevY = false;
    private boolean prevX = false;

    private final double SHOOTER_RPM_SHORT = 1400.0; // 28x2786/60 //28
    private final double SHOOTER_RPM_LONG = 1840; //36?
    private final double SHOOTER_RPM_CLEAR = -1000;

    private double controller1Speed = 1;

    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    private long nowMs() {
        return System.currentTimeMillis();
    }

    @Override
    public void runOpMode() {
        // init();
        robot.init(hardwareMap);
        // start();

        waitForStart();
        while (opModeIsActive()) {
            // loop();

            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean y = gamepad1.y;
            boolean x = gamepad1.x;



            if (gamepad1.left_bumper){
                robot.motorintake.setPower(-1.0);
            } else if (gamepad1.left_trigger > 0){
                robot.motorintake.setPower(1.0);
            } else {
                robot.motorintake.setPower(0);
            }

            if (a && !prevA) {
                robot.startShooterAtRPM(SHOOTER_RPM_SHORT);
            }
            else if (b && !prevB) {
                robot.startShooterAtRPM(SHOOTER_RPM_LONG);

            }

            else if (x && !prevX) {
                robot.startShooterAtRPM(SHOOTER_RPM_CLEAR);
            }

            else if (y && !prevY) {
                robot.stopShooter();
            }

            prevA = a;
            prevB = b;
            prevY = y;
            prevX = x;

            if (gamepad1.dpad_left) {
                robot.startShooterAtRPM(SHOOTER_RPM_SHORT);
                telemetry.addData("intakePower", robot.motorintake.getPower());
                telemetry.addData("shooter velocity", robot.motorshoot.getVelocity());
                telemetry.addData("shooter tpr", robot.motorshoot.getMotorType().getTicksPerRev());
                telemetry.update();
                robot.autoShoot(1430);

            }

            else if (gamepad1.dpad_right) {
                robot.startShooterAtRPM(SHOOTER_RPM_LONG);
                telemetry.addData("intakePower", robot.motorintake.getPower());
                telemetry.addData("shooter velocity", robot.motorshoot.getVelocity());
                telemetry.addData("shooter tpr", robot.motorshoot.getMotorType().getTicksPerRev());
                telemetry.update();
                robot.autoShoot(1785);
            }
            else{
                robot.stopShooter();
            }

            double x_dir = gamepad1.left_stick_x * 1.3 * controller1Speed;
            double y_dir = -gamepad1.left_stick_y * 1.3 * controller1Speed;
            double turn = -gamepad1.right_stick_x * 1.3 * controller1Speed;

            double flPower = x_dir + y_dir + turn;
            double blPower = y_dir - x_dir + turn;
            double frPower = y_dir - x_dir - turn;
            double brPower = y_dir + x_dir - turn;

            robot.setDrivePower(flPower, frPower, blPower, brPower);

            if (gamepad1.dpad_down) {
                controller1Speed = 0.3;
            } else {
                controller1Speed = 1;
            }

            idle();

            // --- TELEMETRY ---
            telemetry.addData("intakePower", robot.motorintake.getPower());
            telemetry.addData("shooter velocity", robot.motorshoot.getVelocity());
            telemetry.addData("shooter tpr", robot.motorshoot.getMotorType().getTicksPerRev());
            telemetry.update();

        }


    }

}


