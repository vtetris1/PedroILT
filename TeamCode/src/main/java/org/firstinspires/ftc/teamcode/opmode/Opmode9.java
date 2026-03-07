package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.hardware.robotHardware.FLAPPER_2_CLOSE;
import static org.firstinspires.ftc.teamcode.hardware.robotHardware.FLAPPER_2_OPEN;
import static org.firstinspires.ftc.teamcode.hardware.robotHardware.FLAPPER_3_CLOSE;
import static org.firstinspires.ftc.teamcode.hardware.robotHardware.FLAPPER_3_OPEN;
import static org.firstinspires.ftc.teamcode.hardware.robotHardware.GATE_CLOSE;
import static org.firstinspires.ftc.teamcode.hardware.robotHardware.GATE_OPEN;
import static org.firstinspires.ftc.teamcode.hardware.robotHardware.INTAKE_POWER_INTAKE;
import static org.firstinspires.ftc.teamcode.hardware.robotHardware.INTAKE_POWER_OUTTAKE;
import static org.firstinspires.ftc.teamcode.hardware.robotHardware.INTAKE_POWER_STOP;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.hardware.robotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseItem;

import java.util.ArrayList;
import java.util.function.Supplier;

@Configurable
@TeleOp(name = "Opmode9")
public class Opmode9 extends LinearOpMode {
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
    private static final double TARGET_HEIGHT_INCH = 20;
    private static final double MAX_HEIGHT_INCH = 21;
    // Constants for GoBilda 5203 30 rpm motor
    // private static final double TICKS_PER_REVOLUTION_ELEVATOR_MOTOR = 5281.1;        // 30RPM motor constant PPR given by GoBilda

    // The following ratio is added based on actual test with a target height.
    // Actual rising height is 18 inches and raised height is 36 inch.
    // However, the cleared space height is about 37 inch since both PODs extended downward for close to 1.0 inch.
    // Target to rise up 1.0 inch more to reach 37.0 inch, 1.0 inch below the  maximum 38 inch.
    // The extra 1.0 inch is needed so that a complete 18 inch height space is created after both PODs are extended down fully.
    private static final double RATIO_60RPM = 1 + (1.0/TARGET_HEIGHT_INCH);
// 60RPM motor constant PPR given by GoBilda and scaled by actual testing results.
//    private static final double TICKS_PER_REVOLUTION_ELEVATOR_MOTOR = 2786.2 * RATIO_60RPM;

    private static final double RATIO_84RPM = 1 + (1.0/TARGET_HEIGHT_INCH);
    // 84RPM motor constant PPR given by GoBilda and scaled by actual testing results.
    private static final double TICKS_PER_REVOLUTION_ELEVATOR_MOTOR = 1992.6 * RATIO_84RPM;
    private static final double CIRCUMFERENCE_HTD5M_PULLEY_24T_IN_INCHES = 5 * 24 / 25.4;
    private static final double ELEVATOR_RISING_UP_POWER = 0.95;
    private static final double ELEVATOR_GOING_DOWN_POWER = 0.4;
    private static double ELEVATOR_GOING_DOWN_INCHES = -0.25;
    private static double ELEVATOR_GOING_DOWN_TICKS =
            (ELEVATOR_GOING_DOWN_INCHES / CIRCUMFERENCE_HTD5M_PULLEY_24T_IN_INCHES * TICKS_PER_REVOLUTION_ELEVATOR_MOTOR);

    private static double elevator_target_height_inches = TARGET_HEIGHT_INCH;
    private static double elevator_target_height_ticks =
            (elevator_target_height_inches / CIRCUMFERENCE_HTD5M_PULLEY_24T_IN_INCHES * TICKS_PER_REVOLUTION_ELEVATOR_MOTOR); //fix should be about 10,500


    private double controller1Speed = 1.0;
    ElapsedTime controller1SpeedChangeTimer = new ElapsedTime();

    
    ElapsedTime triggerTimer = new ElapsedTime();
    ElapsedTime intakeStartTimer = new ElapsedTime();
    ElapsedTime shootTimer1 = new ElapsedTime();
    ElapsedTime shootTimer2 = new ElapsedTime();
    ElapsedTime triggerTimer2 = new ElapsedTime();
    ElapsedTime shootTimer3 = new ElapsedTime();
    ElapsedTime shooterSpeedChangeTimer = new ElapsedTime();
    ElapsedTime flapper2Timer = new ElapsedTime();
    ElapsedTime flapper3Timer = new ElapsedTime();
    static final double FLAPPER_2_BUTTON_TIME = 0.2;
    static final double FLAPPER_2_CLOSE_TIME = 0.5;
    static final double FLAPPER_3_BUTTON_TIME = 0.2;
    static final double FLAPPER_3_CLOSE_TIME = 0.5;
    private boolean flapper2ManualTriggered = false;
    private boolean flapper3ManualTriggered = false;

    private boolean localizationActive = false;

    static final double INTAKE_START_TIME = 0.8;
    static final double SHOOT_1_TIME = 0.7;
    static final double SHOOT_2_TIME = 0.5;
    static final double TRIGGER_2_TIME = 0.5;
    static final double SHOOT_3_TIME = 0.3;
    static final double TRIGGER_3_TIME = 0.5;


    protected final Pose park_place = new Pose(38, 25, Math.toRadians(180));
    static final double TRIGGER_SHOOT_TIME = 0.5;

    static final double SPEED_CHANGE_TIME = 0.15; // seconds to handle physical button/key natural time

 
    public enum SHOOT_STATES {
        IDLE,
        SPIN_UP1,
        START_INTAKE1,
        SHOOT_1ST,
        SPIN_UP2,
        SHOOT_2ND,
        SPIN_UP3,
        SHOOT_3RD,
    }
    // 🔹 UPDATED STATE MACHINE

    private SHOOT_STATES shootState = SHOOT_STATES.IDLE;
    private boolean bShootRequested = false;

    final double STOP_SPEED = 0.0;

    private boolean toggleSpeed = false;

    boolean bTriggerEnabled = false;

    // Constants for GoBilda 5203 6000 rpm motor
    // Constants for GoBilda 5203 6000 rpm motor
    static final double TICKS_PER_REVOLUTION = 28.0;
    static final double MAX_TICKS_PER_SEC = 2800.0;

    private final double SHOOTER_RPM_SHORT = 2700.0; //3000 // 28x2786/60 //28
    private final double SHOOTER_RPM_LONG = 3350.0; //36?

    final double SHOOTER_TARGET_RANGE = 20;

    private double shooter_target_rpm = SHOOTER_RPM_SHORT;
    private double shooter_target_ticks = shooter_target_rpm * TICKS_PER_REVOLUTION / 60;
    private double shooter_target_ticks_low= (shooter_target_rpm - SHOOTER_TARGET_RANGE) * TICKS_PER_REVOLUTION / 60;


    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private final double turretPower = 0.9;
    private boolean turretMode = false;
    private double turretFactor = 1;




    private long nowMs() {
        return System.currentTimeMillis();
    }


    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    ArrayList<PoseItem> poseList = new ArrayList<PoseItem>() ;

    public void relocalizeWithLimelight() {
        LLResult result = robot.limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            // Snap Pinpoint to the Vision data
            robot.pinpoint.setPosition(new Pose2D(
                    DistanceUnit.MM,
                    botpose.getPosition().x * 1000,
                    botpose.getPosition().y * 1000,
                    AngleUnit.DEGREES,
                    botpose.getOrientation().getYaw()
            ));
        }
    }

    public void localizationUpdate(){

        LLResult result = robot.limelight.getLatestResult();
        robot.limelight.updateRobotOrientation(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        robot.pinpoint.update();

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();

            double fieldX = botpose.getPosition().x * 1000;
            double fieldY = botpose.getPosition().y * 1000;
            double fieldHeading = botpose.getOrientation().getYaw();

            robot.pinpoint.setPosition(new Pose2D(DistanceUnit.MM, fieldX, fieldY, AngleUnit.DEGREES, fieldHeading));
        }

        Pose2D currentPos = robot.pinpoint.getPosition();
        follower.setPose(new Pose(
                currentPos.getX(DistanceUnit.INCH),
                currentPos.getY(DistanceUnit.INCH),
                Math.toRadians(currentPos.getHeading(AngleUnit.DEGREES))
        ));


        telemetry.addData("Status", robot.pinpoint.getDeviceStatus());
        telemetry.addData("x", currentPos.getX(DistanceUnit.MM));
        telemetry.addData("y", currentPos.getY(DistanceUnit.MM));
        telemetry.addData("heading", currentPos.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    public void updateOdometer() {
        robot.pinpoint.update();
        Pose2D pos = robot.pinpoint.getPosition();
        follower.setPose(new Pose(
                pos.getX(DistanceUnit.INCH),
                pos.getY(DistanceUnit.INCH),
                pos.getHeading(AngleUnit.RADIANS)
        ));
    }


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        bShootRequested = false;
        waitForStart();

        controller1SpeedChangeTimer.reset();
        triggerTimer.reset();

        while (opModeIsActive()) {

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


            updateOdometer();
            follower.update();

            //intake/middle
            if(gamepad2.dpad_down) {
                robot.motorintake.setPower(0.5);
                sleep(50);
            }
            else if (gamepad1.left_bumper){
                robot.motorintake.setPower(INTAKE_POWER_INTAKE);
            } else if (gamepad1.left_trigger > 0.5){
                robot.motorintake.setPower(INTAKE_POWER_OUTTAKE);
            } else if (!bShootRequested) {
                robot.motorintake.setPower(INTAKE_POWER_STOP);
            }
            if (gamepad1.dpad_left) {
                if (controller1SpeedChangeTimer.seconds() >= SPEED_CHANGE_TIME && !toggleSpeed) {
                    controller1SpeedChangeTimer.reset();
                    toggleSpeed = true;
                }
                else{
                    toggleSpeed = false;
                    controller1SpeedChangeTimer.reset();
                    controller1Speed = 1;

                }
            }

            //turret
            if (gamepad2.dpad_left){
                robot.motorturret.setPower(turretPower * turretFactor);
            } else if (gamepad2.dpad_right){
                robot.motorturret.setPower(-turretPower * turretFactor);
            } else {
                robot.motorturret.setPower(0);
            }

            // flapper control


            // trigger to launch the single artifact

            if (localizationActive){
                relocalizeWithLimelight();
            }

            if (gamepad2.dpad_up && gamepad2.a && localizationActive) {
                driveTo(park_place);
            }
            else if (gamepad2.dpad_up) {

                if (!localizationActive){
                    localizationActive = true;
                    robot.limelight.start();
    //                localizationUpdate();

                }
                else{
                    localizationActive = false;
                    robot.limelight.stop();
                }


            }
            else if (gamepad2.right_trigger > 0.5) {
                bShootRequested = true;
            }
            else if (gamepad2.right_bumper) {
                bShootRequested = false;
            }
            else if (bTriggerEnabled && triggerTimer.seconds() >= TRIGGER_SHOOT_TIME) {
                if (robot.motorshoot.getVelocity() < 1000){
                    robot.gate.setPosition(GATE_CLOSE);
                    bTriggerEnabled = false;
                }
            }

            // left trigger to manually close flapper 3
            // boolean flag is required in order not to mess up shooterStateMachine.
            if (gamepad2.right_trigger > 0.5) {
                if (flapper3Timer.seconds() > FLAPPER_3_BUTTON_TIME) {
                    flapper3Timer.reset();
                    robot.flapper3.setPosition(FLAPPER_3_CLOSE);
                    flapper2ManualTriggered = true;
                }
            }
            else if (flapper2ManualTriggered) {
                if (flapper3Timer.seconds() > FLAPPER_3_CLOSE_TIME) {
                    robot.flapper3.setPosition(FLAPPER_3_OPEN);
                    flapper2ManualTriggered = false;
                }
            }

            // left trigger to manually close flapper 2
            // boolean flag is required in order not to mess up shooterStateMachine.
            if (gamepad2.left_bumper) {
                if (flapper2Timer.seconds() > FLAPPER_2_BUTTON_TIME) {
                    flapper2Timer.reset();
                    robot.flapper2.setPosition(FLAPPER_2_CLOSE);
                    flapper3ManualTriggered = true;
                }
            }
            else if (flapper3ManualTriggered) {
                if (flapper2Timer.seconds() > FLAPPER_2_CLOSE_TIME) {
                    robot.flapper2.setPosition(FLAPPER_2_OPEN);
                    flapper3ManualTriggered = false;
                }
            }

            // Shooter speed control
            if (gamepad2.x){
                // Far side
                if (shooterSpeedChangeTimer.seconds() > SPEED_CHANGE_TIME) {
                    shooterSpeedChangeTimer.reset();
                    shooter_target_rpm = SHOOTER_RPM_LONG;
                }
                updateShooterTargetRpm(shooter_target_rpm);
            }
            else if (gamepad2.b){
                // Near side
                if (shooterSpeedChangeTimer.seconds() > SPEED_CHANGE_TIME) {
                    shooterSpeedChangeTimer.reset();
                    shooter_target_rpm = SHOOTER_RPM_SHORT;
                }
                updateShooterTargetRpm(shooter_target_rpm);
            }
            else if (gamepad2.y) {
                // Minor adjustment
                if (shooterSpeedChangeTimer.seconds() > SPEED_CHANGE_TIME) {
                    shooterSpeedChangeTimer.reset();
                    shooter_target_rpm += 50;
                }
                updateShooterTargetRpm(shooter_target_rpm);
            }
            else if (gamepad2.a){
                if (shooterSpeedChangeTimer.seconds() > SPEED_CHANGE_TIME) {
                    shooterSpeedChangeTimer.reset();
                    shooter_target_rpm -= 50;
                }
                updateShooterTargetRpm(shooter_target_rpm);
            }

            runShootStateMachine();
            //elevator
            runElevatorStateMachine();
            // --- TELEMETRY ---
//            telemetry.addData("intakePower", robot.motorintake.getPower());
//            telemetry.addData("shooter velocity", (robot.motorshoot.getVelocity() * 2 ));
//            telemetry.addData("shooter tpr", robot.motorshoot.getMotorType().getTicksPerRev());
//            telemetry.addData("shooter rpm", shooter_target_rpm);
//            telemetry.addData("controller1Speed", controller1Speed);
//            telemetry.addData("turret factor", turretFactor);
            telemetry.update();
            idle();
        }
    }

    void updateShooterTargetRpm(double target_rpm) {
        shooter_target_rpm = target_rpm;
        shooter_target_ticks = Math.max(0, Math.min(MAX_TICKS_PER_SEC,
                shooter_target_rpm * TICKS_PER_REVOLUTION / 60));
        shooter_target_ticks_low = Math.max(0, Math.min(MAX_TICKS_PER_SEC,
                (shooter_target_rpm - SHOOTER_TARGET_RANGE) * TICKS_PER_REVOLUTION / 60));
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
                    elevator_target_height_ticks = (elevator_target_height_inches / CIRCUMFERENCE_HTD5M_PULLEY_24T_IN_INCHES * TICKS_PER_REVOLUTION_ELEVATOR_MOTOR);
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
//        telemetry.addData("parkState", parkState);
//        telemetry.addData("elevatorInches", elevator_target_height_inches);
//        telemetry.addData("elevatorTicks", elevator_target_height_ticks);
//        telemetry.addData("elevatorPos", robot.elevator.getCurrentPosition());
    }

    void goDownSlowly() {
        robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.elevator.setTargetPosition((int) (ELEVATOR_GOING_DOWN_TICKS));
        robot.elevator.setPower(ELEVATOR_GOING_DOWN_POWER);
        robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void runShootStateMachine() {

        switch (shootState) {
            case IDLE:
                if (bShootRequested) {
                    shootState = SHOOT_STATES.SPIN_UP1;
                }
                break;
            case SPIN_UP1:
                if (bShootRequested) {
                    robot.gate.setPosition(GATE_OPEN);
                    if (robot.motorshoot.getVelocity() > shooter_target_ticks_low) {
                        shootState = SHOOT_STATES.START_INTAKE1;
                        intakeStartTimer.reset();
                    }
                }
                break;
            case START_INTAKE1:
                if (bShootRequested) {

                    robot.motorintake.setPower(0.5);
                    sleep(30);

                    robot.motorintake.setPower(INTAKE_POWER_INTAKE);



                    if (intakeStartTimer.seconds() > INTAKE_START_TIME) {
                        shootState = SHOOT_STATES.SHOOT_1ST;
                        shootTimer1.reset();
                    }
                }
                break;

            case SHOOT_1ST:
                if (bShootRequested) {
                    // Shoot 1st artifact by intake stage 3 flapper
                    robot.flapper3.setPosition(FLAPPER_3_CLOSE);
                    if (shootTimer1.seconds() > SHOOT_1_TIME) {
                        shootState = SHOOT_STATES.SPIN_UP2;
                        robot.flapper3.setPosition(FLAPPER_3_OPEN);
                    }
                    sleep(100);
                }
                break;

            case SPIN_UP2:
                if (bShootRequested) {
                    robot.gate.setPosition(GATE_OPEN);
                    if (robot.motorshoot.getVelocity() > shooter_target_ticks_low) {
                        shootState = SHOOT_STATES.SHOOT_2ND;
                        shootTimer2.reset();
                        robot.flapper3.setPosition(FLAPPER_3_CLOSE);
                        robot.flapper2.setPosition(FLAPPER_2_CLOSE);
                    }
                }
                break;

            case SHOOT_2ND:
                if (bShootRequested) {
                    // Keep stage 3 flapper closed.
                    // Shoot 2nd artifact by intake stage 2 flapper if target rpm reached
                    robot.flapper3.setPosition(FLAPPER_3_CLOSE);
                    robot.flapper2.setPosition(FLAPPER_2_CLOSE);
                    robot.motorintake.setPower(INTAKE_POWER_INTAKE);
                    if (shootTimer2.seconds() > SHOOT_2_TIME) {
                        shootState = SHOOT_STATES.SPIN_UP3;

                        robot.flapper3.setPosition(FLAPPER_3_OPEN);
                        robot.flapper2.setPosition(FLAPPER_2_OPEN);
                        sleep(100);
                    }
                }
                break;
            case SPIN_UP3:
                if (bShootRequested) {
                    robot.gate.setPosition(GATE_OPEN);
                    if (robot.motorshoot.getVelocity() > shooter_target_ticks_low) {
                        shootState = SHOOT_STATES.SHOOT_3RD;
                        shootTimer3.reset();
                        robot.flapper3.setPosition(FLAPPER_3_CLOSE);
                        robot.flapper2.setPosition(FLAPPER_2_CLOSE);
                    }
                }
                break;

            case SHOOT_3RD:
                if (bShootRequested) {
                    robot.flapper3.setPosition(FLAPPER_3_CLOSE);
                    robot.flapper2.setPosition(FLAPPER_2_CLOSE);
                    robot.motorintake.setPower(INTAKE_POWER_INTAKE);
                    if (shootTimer3.seconds() > SHOOT_3_TIME) {
                        shootState = SHOOT_STATES.IDLE;
                        bShootRequested = false;
                        robot.motorintake.setPower(INTAKE_POWER_STOP);
                    }
                }
                break;
        }
        if (bShootRequested) {
            robot.gate.setPosition(GATE_OPEN);
            robot.motorshoot.setVelocity(shooter_target_ticks);
        }
        else {
            shootState = SHOOT_STATES.IDLE;
            robot.gate.setPosition(GATE_CLOSE);
            robot.motorshoot.setVelocity(STOP_SPEED);

            robot.flapper3.setPosition(FLAPPER_3_OPEN);
            robot.flapper2.setPosition(FLAPPER_2_OPEN);
        }
    }

    public void driveTo(Pose target) {
        follower.followPath(follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), target))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), target.getHeading())
                .build(), true);
    }

}

