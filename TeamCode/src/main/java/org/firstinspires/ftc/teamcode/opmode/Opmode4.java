package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.robotHardware;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "Opmode4")
public class Opmode4 extends LinearOpMode {
    robotHardware robot = new robotHardware();

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevY = false;
    private boolean prevX = false;

    public enum PARK_STATES {
        INIT,
        RISING_UP,
        RISED,
        // going down slowly (0.5 inch per command) in order to reset slides back to the start position
        GOING_DOWN,

    }
    private PARK_STATES parkState = PARK_STATES.INIT;
    // Constants for GoBilda 5203 30 rpm motor
    private static final double TICKS_PER_REVOLUTION_30RPM = 5281.1;        // constant given by GoBilda
    private static final double CIRCUMFERENCE_HTD5M_PULLEY_24T_IN_INCHES = 5 * 24 / 25.4;
    private static final double TARGET_HEIGHT_INCH = 20;
    private static final double MAX_HEIGHT_INCH = 21;
    private static final double ELEVATOR_RISING_UP_POWER = 0.95;
    private static final double ELEVATOR_GOING_DOWN_POWER = 0.4;
    private static double ELEVATOR_GOING_DOWN_INCHES = -0.25;
    private static double ELEVATOR_GOING_DOWN_TICKS =
            (ELEVATOR_GOING_DOWN_INCHES / CIRCUMFERENCE_HTD5M_PULLEY_24T_IN_INCHES * TICKS_PER_REVOLUTION_30RPM);

    private static double elevator_target_height_inches = TARGET_HEIGHT_INCH;
    private static double elevator_target_height_ticks =
            (elevator_target_height_inches / CIRCUMFERENCE_HTD5M_PULLEY_24T_IN_INCHES * TICKS_PER_REVOLUTION_30RPM);
    // Constants for GoBilda 5203 6000 rpm motor
    static final double TICKS_PER_REVOLUTION = 28.0;
    static final double MAX_TICKS_PER_SEC = 2800.0;




    private final double SHOOTER_RPM_SHORT = 3000.0; // 28x2786/60 //28
    private final double SHOOTER_RPM_LONG = 3880; //36?
    private final double SHOOTER_RPM_CLEAR = -1000;
    private final double SHOOTER_CHANGE = 200;
    private double rpm = SHOOTER_RPM_SHORT;



    private double controller1Speed = 1.0;
    ElapsedTime controller1SpeedChangeTimer = new ElapsedTime();

    ElapsedTime triggerTimer = new ElapsedTime();
    static final double TRIGGER_READY = 0.6;
    static final double TRIGGER_SHOOT = 0.2;
    static final double TRIGGER_SHOOT_TIME = 0.5;

    static final double SPEED_CHANGE_TIME = 0.15; // seconds to handle physical button/key natural time
    boolean bTriggerEnabled = false;
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

        controller1SpeedChangeTimer.reset();
        triggerTimer.reset();

        while (opModeIsActive()) {
            boolean a = gamepad2.a;
            boolean b = gamepad2.b;
            boolean y = gamepad2.y;
            boolean x = gamepad2.x;

//DRIVING
            double x_dir = gamepad1.left_stick_x * controller1Speed;
            double y_dir = -gamepad1.left_stick_y * controller1Speed;
            double turn = gamepad1.right_stick_x * controller1Speed;

            double flPower = x_dir + y_dir + turn;
            double blPower = y_dir - x_dir + turn;
            double frPower = y_dir - x_dir - turn;
            double brPower = y_dir + x_dir - turn;

            // The following code it ensure power of all motors are scaling down properly
            // so that the power is within the range [-1.0, 1.0]
            // Direct limit check could cause imbalanced power levels...
            double scaling = Math.max(1.0,
                    Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                            Math.max(Math.abs(blPower), Math.abs(brPower))));
            flPower = flPower / scaling;
            frPower = frPower / scaling;
            blPower = blPower / scaling;
            brPower = brPower / scaling;

            robot.setDrivePower(flPower, frPower, blPower, brPower);

            // The following logic will have controller1Speed changing between 0.2 and 1 repeatedly once dpad_down is released.
            // dpad_down could be read MULTIPLE times, once dpad_down is released controller1Speed will be changed to 1.0.
            // Hence controller1Speed is not maintained.
            // It is better to use a button to change the speed back OR alternatively to adjust speed on the fly.
            if (gamepad1.dpad_left) {
                if (controller1SpeedChangeTimer.seconds() >= SPEED_CHANGE_TIME) {
                    controller1SpeedChangeTimer.reset();
                    controller1Speed = 0.2;
                }
            } else if (gamepad1.dpad_right) {
                if (controller1SpeedChangeTimer.seconds() >= SPEED_CHANGE_TIME) {
                    controller1SpeedChangeTimer.reset();
                    controller1Speed = 1;
                }
            } else if (gamepad1.dpad_up) {
                if (controller1SpeedChangeTimer.seconds() >= SPEED_CHANGE_TIME) {
                    controller1SpeedChangeTimer.reset();
                    controller1Speed = Math.min(1.0, controller1Speed + 0.2);
                }
            } else if (gamepad1.dpad_down) {
                if (controller1SpeedChangeTimer.seconds() >= SPEED_CHANGE_TIME) {
                    controller1SpeedChangeTimer.reset();
                    controller1Speed = Math.max(0.2, controller1Speed - 0.2);
                }
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
                sleep(50);  // this is a blocking call which should be avoided in teleop mode
            }
            if(x && !prevX){
                rpm -= SHOOTER_CHANGE;
                sleep(50);  // this is a blocking call which should be avoided in teleop mode.
            }

            if (y && !prevY){
                robot.autoShoot(rpm);
                //find a solution for this
            }
            if (gamepad2.right_bumper){
                robot.autoShoot(SHOOTER_RPM_SHORT);
            }
            else if (gamepad2.right_trigger > 0.5){
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
            if (gamepad2.dpad_left){
                robot.motorturret.setPower(0.5);
            } else if (gamepad2.dpad_right){
                robot.motorturret.setPower(-0.5);
            } else {
                robot.motorturret.setPower(0);
            }

            if (gamepad2.left_bumper){
                robot.servoL.setPower(-1);
                robot.servoR.setPower(1);
            } else if (gamepad2.left_trigger > 0.8){
                robot.servoL.setPower(1);
                robot.servoR.setPower(-1);
            } else {
                robot.servoL.setPower(0);
                robot.servoR.setPower(0);
            }

            // trigger to launch the single artifact
            if (gamepad2.dpad_up) {
                if (triggerTimer.seconds() >= TRIGGER_SHOOT_TIME) {
                    triggerTimer.reset();
                    robot.trigger.setPosition(TRIGGER_SHOOT);
                    bTriggerEnabled = true;
                }
            }

            else if(gamepad2.dpad_down) {
                robot.trigger.setPosition(TRIGGER_READY);
                bTriggerEnabled = false;
            }

            else {
                if (bTriggerEnabled && triggerTimer.seconds() >= TRIGGER_SHOOT_TIME) {
                    robot.trigger.setPosition(TRIGGER_READY);
                    bTriggerEnabled = false;
                }
            }




            //elevator
            runElevatorStateMachine();
            // --- TELEMETRY ---
            telemetry.addData("intakePower", robot.motorintake.getPower());
            telemetry.addData("shooter velocity", robot.motorshoot.getVelocity());
            telemetry.addData("shooter tpr", robot.motorshoot.getMotorType().getTicksPerRev());
            telemetry.addData("projected rpm", rpm);
            telemetry.addData("controller1Speed", controller1Speed);
            telemetry.update();
            idle();
        }


    }

    void runElevatorStateMachine() {
        switch (parkState) {
            case INIT:
                if (gamepad2.left_stick_y <= -0.5 && gamepad2.right_stick_y <= -0.5) {
                    //robot.elevator.(); negative ticks
                    //2786.2 ticks per rotation
                    /* SP: This is unnecessary since the elevator mode was initialized once during .init().
                    robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    */
                    robot.elevator.setTargetPosition((int)(elevator_target_height_ticks));
                    robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.elevator.setPower(ELEVATOR_RISING_UP_POWER);
                    parkState = PARK_STATES.RISING_UP;
                }
                else if (gamepad2.left_stick_y >= 0.5 && gamepad2.right_stick_y >= 0.5) {
                    // both joysticks are flipped downward: reset encode, going down slowly 0.5 inch per command, and reset
                    goDownSlowly();
                    parkState = PARK_STATES.GOING_DOWN;
                }
                break;
            case RISING_UP:
                if (!robot.elevator.isBusy()) {
                    robot.elevator.setPower(0);
                    parkState = PARK_STATES.RISED;
                }
                break;
            case RISED:
                if (gamepad2.left_stick_y <= -0.5 || gamepad2.right_stick_y <= -0.5) {
                    elevator_target_height_inches = Math.min(elevator_target_height_inches + 1, TARGET_HEIGHT_INCH);
                    elevator_target_height_ticks = (elevator_target_height_inches / CIRCUMFERENCE_HTD5M_PULLEY_24T_IN_INCHES * TICKS_PER_REVOLUTION_30RPM);
                    if (elevator_target_height_inches < TARGET_HEIGHT_INCH) {
                        robot.elevator.setTargetPosition((int) (elevator_target_height_ticks));
                        robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.elevator.setPower(ELEVATOR_RISING_UP_POWER);
                        parkState = PARK_STATES.RISING_UP;
                    }
                }
                else if (gamepad2.left_stick_y >= 0.5 && gamepad2.right_stick_y >= 0.5) {
                    // both joysticks are flipped downward: reset encode, going down slowly 0.5 inch per command, and reset
                    goDownSlowly();
                    parkState = PARK_STATES.GOING_DOWN;
                }
                break;
            case GOING_DOWN:
                if (!robot.elevator.isBusy()) {
                    robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.elevator.setPower(0);
                    parkState = PARK_STATES.INIT;
                }
                break;
        }
        telemetry.addData("parkState", parkState);
        telemetry.addData("elevatorInches", elevator_target_height_inches);
        telemetry.addData("elevatorTicks", elevator_target_height_ticks);
        telemetry.addData("elevatorPos", robot.elevator.getCurrentPosition());
    }

    void goDownSlowly() {
        robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.elevator.setTargetPosition((int) (ELEVATOR_GOING_DOWN_TICKS));
        robot.elevator.setPower(ELEVATOR_GOING_DOWN_POWER);
        robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}

