package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.hardware.robotHardware.GATE_CLOSE;
import static org.firstinspires.ftc.teamcode.hardware.robotHardware.GATE_OPEN;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.hardware.MedianFilter5;
import org.firstinspires.ftc.teamcode.hardware.robotHardware;

import java.util.function.Supplier;

@Configurable
@Disabled
@TeleOp(name = "ParkTest")
public class ParkTest extends LinearOpMode {
    robotHardware robot = new robotHardware();

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevY = false;
    private boolean prevX = false;

    public enum ELEVATOR_STATES {
        INIT,
        RISING_UP,
        RISED,
        // going down slowly (0.5 inch per command) in order to reset slides back to the start position
        GOING_DOWN,

    }
    private ELEVATOR_STATES elevatorState = ELEVATOR_STATES.INIT;
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

    // Constants for GoBilda 5203 6000 rpm motor
    static final double TICKS_PER_REVOLUTION = 28.0;
    static final double MAX_TICKS_PER_SEC = 2800.0;




    private final double SHOOTER_RPM_SHORT = 2600.0; //3000 // 28x2786/60 //28
    private final double SHOOTER_RPM_LONG = 3050; //36?
    private final double SHOOTER_RPM_CLEAR = -500;
    private final double SHOOTER_CHANGE = 200;
    private double rpm = SHOOTER_RPM_SHORT;



    private double controller1Speed = 1.0;
    ElapsedTime controller1SpeedChangeTimer = new ElapsedTime();

    ElapsedTime triggerTimer = new ElapsedTime();
    static final double TRIGGER_SHOOT_TIME = 0.5;

    static final double SPEED_CHANGE_TIME = 0.15; // seconds to handle physical button/key natural time
    boolean bTriggerEnabled = false;
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


    Limelight3A limelight;

    /* ---------------- Targets ---------------- */
    static final double TARGET_HEADING = 180.0;
    static final double FRONT_MIN = 24.5;
    static final double FRONT_MAX = 25.0;
    static final double FRONT_MID = (FRONT_MIN + FRONT_MAX) / 2.0;
    static final double RIGHT_MIN = 30.0;
    static final double RIGHT_MAX = 31.0;
    static final double RIGHT_MID = (RIGHT_MIN + RIGHT_MAX) / 2.0;

    /* ---------------- Controller Gains ---------------- */
    double kP_distance = 0.05;
    double kP_heading  = 0.015;
    double MAX_POWER = 0.30;

    /* ---------------- State Machine ---------------- */

    enum PARK_STATES {
        INIT,
        APPROACHING,
        COMPLETE
    }

    PARK_STATES parkState = PARK_STATES.INIT;
    MedianFilter5 medianFilterF = new MedianFilter5();
    MedianFilter5 medianFilterR = new MedianFilter5();
    MedianFilter5 medianFilterL = new MedianFilter5();

    double filteredF = 30;
    double filteredR = 30;
    double filteredL = 30;

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
            double x_dir = -gamepad1.left_stick_x * controller1Speed;
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

            if (gamepad1.dpad_left) {

                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    Pose3D pose = result.getBotpose();
                    //limelight.

                    telemetry.addData("x", "%.2f m", pose.getPosition().x);
                    telemetry.addData("y",      "%.2f m", pose.getPosition().y);
                    telemetry.addData("heading", "%.2f deg", pose.getOrientation().getYaw());
                } else {
                    telemetry.addData("e:", "Tag 24 Not Found");
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
                robot.autoShoot(SHOOTER_RPM_SHORT);
                //find a solution for this
            }
            if (gamepad2.right_bumper){
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
                robot.motorturret.setPower(turretPower * turretFactor);
            } else if (gamepad2.dpad_right){
                robot.motorturret.setPower(-turretPower * turretFactor);
            } else {
                robot.motorturret.setPower(0);
            }


            // trigger to launch the single artifact
            if (gamepad2.dpad_up) {
                if (triggerTimer.seconds() >= TRIGGER_SHOOT_TIME) {
                    triggerTimer.reset();
                    robot.gate.setPosition(GATE_OPEN);
                    bTriggerEnabled = true;
                }
            }

            else if(gamepad2.dpad_down) {
                turretMode = !turretMode;
                if(turretMode){
                    turretFactor = 0.33;
                    sleep(500);
                }
                else{
                    turretFactor = 1;
                    sleep(500);
                }
            }

            else {
                if ((bTriggerEnabled && triggerTimer.seconds() >= TRIGGER_SHOOT_TIME) && (robot.motorshoot.getVelocity() < 1000)){
                    robot.gate.setPosition(GATE_CLOSE);
                    bTriggerEnabled = false;
                }
            }

            //precisionParkStateMachine
            runPrecisionParkStateMachine();
            //elevator
            runElevatorStateMachine();
            // --- TELEMETRY ---
            telemetry.addData("intakePower", robot.motorintake.getPower());
            telemetry.addData("shooter velocity", (robot.motorshoot.getVelocity() * 2 ));
            telemetry.addData("shooter tpr", robot.motorshoot.getMotorType().getTicksPerRev());
            telemetry.addData("projected rpm", rpm);
            telemetry.addData("controller1Speed", controller1Speed);
            telemetry.addData("turret factor", turretFactor);
            telemetry.addData("heading", String.format("%.01f degree", robot.getCurrentYaw()));
            telemetry.addData("distanceF", String.format("%.01f in", filteredF));
            telemetry.addData("distanceR", String.format("%.01f in", filteredR));
            telemetry.addData("distanceL", String.format("%.01f in", filteredL));

            telemetry.update();
            idle();
        }


    }

    void runElevatorStateMachine() {
        switch (elevatorState) {
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
                    elevatorState = ELEVATOR_STATES.RISING_UP;
                }
                else if (gamepad2.left_stick_y >= 0.5 && gamepad2.right_stick_y >= 0.5) {
                    // both joysticks are flipped downward: reset encode, going down slowly 0.5 inch per command, and reset
                    goDownSlowly();
                    elevatorState = ELEVATOR_STATES.GOING_DOWN;
                }
                break;
            case RISING_UP:
                if (!robot.elevator.isBusy()) {
                    robot.elevator.setPower(0);
                    elevatorState = ELEVATOR_STATES.RISED;
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
                        elevatorState = ELEVATOR_STATES.RISING_UP;
                    }
                }
                else if (gamepad2.left_stick_y >= 0.5 && gamepad2.right_stick_y >= 0.5) {
                    // both joysticks are flipped downward: reset encode, going down slowly 0.5 inch per command, and reset
                    goDownSlowly();
                    elevatorState = ELEVATOR_STATES.GOING_DOWN;
                }
                break;
            case GOING_DOWN:
                if (!robot.elevator.isBusy()) {
                    robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.elevator.setPower(0);
                    elevatorState = ELEVATOR_STATES.INIT;
                }
                break;
        }
        telemetry.addData("elevatorState", elevatorState);
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

    void runPrecisionParkStateMachine() {
        updateDistances();
        switch (parkState) {
            case INIT:
                if (gamepad1.a) {
                    parkState = PARK_STATES.APPROACHING;
                }
                break;
            case APPROACHING:
                double xPower = -computeRightCorrection();
                double yPower = -computeFrontCorrection();
                double turnPower   = computeHeadingCorrection();
                telemetry.addData("powers: ",
                        String.format("x: %.02f; y: %.02f; t: %.02f",
                                xPower, yPower, turnPower));
                driveRobotByPower(xPower, yPower, turnPower);

                if (withinTarget()) {
                    parkState = PARK_STATES.COMPLETE;
                }
                break;
            case COMPLETE:
                driveRobotByPower(0,0,0);
                break;
        }
        telemetry.addData("parkState", parkState);
    }
    /* ---------------- SENSOR UPDATE ---------------- */

    private void updateDistances() {
        double raw = robot.distanceF.getDistance(DistanceUnit.INCH);
        if (raw > 1 && raw < 60) {
            filteredF = medianFilterF.update(raw);
        }

        raw = robot.distanceR.getDistance(DistanceUnit.INCH);
        if (raw > 1 && raw < 60) {
            filteredR = medianFilterR.update(raw);
        }

        raw = robot.distanceL.getDistance(DistanceUnit.INCH);
        if (raw > 1 && raw < 60) {
            filteredL = medianFilterL.update(raw);
        }
    }

    /* ---------------- CONTROLLERS ---------------- */

    private double computeFrontCorrection() {
        if (filteredF >= FRONT_MIN &&
                filteredF <= FRONT_MAX)
            return 0;
        double error = FRONT_MID - filteredF;
        return clip(error * kP_distance);
    }

    private double computeRightCorrection() {
        if (filteredR >= RIGHT_MIN &&
                filteredR <= RIGHT_MAX)
            return 0;
        double error = RIGHT_MID - filteredR;
        return clip(error * kP_distance);
    }

    private double computeHeadingCorrection() {
        double heading = robot.getCurrentYaw();
        double error = normalizeAngle(TARGET_HEADING - heading);
        return clip(error * kP_heading);
    }

    /* ---------------- DRIVE ---------------- */
    private void driveRobotByPower(double x, double y, double turn) {
        double flp = y + x + turn;
        double frp = y - x - turn;
        double blp = y - x + turn;
        double brp = y + x - turn;

        robot.motorfl.setPower(flp);
        robot.motorfr.setPower(frp);
        robot.motorbl.setPower(blp);
        robot.motorbr.setPower(brp);
    }
    /* ---------------- HELPERS ---------------- */

    private boolean withinTarget() {

        return filteredF >= FRONT_MIN &&
                filteredF <= FRONT_MAX &&
                filteredR >= RIGHT_MIN &&
                filteredR <= RIGHT_MAX;
    }

    private double clip(double v) {
        return Math.max(-MAX_POWER,
                Math.min(MAX_POWER, v));
    }

    private double normalizeAngle(double angle) {

        while(angle > 180) angle -= 360;
        while(angle < -180) angle += 360;

        return angle;
    }
}

