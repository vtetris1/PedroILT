package org.firstinspires.ftc.teamcode.hardware;


import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
*/

public class robotHardware {
    HardwareMap hwMap =  null;

    public IMU imu;
    public DcMotor motorfl = null;
    public DcMotor motorfr = null;
    public DcMotor motorbr = null;
    public DcMotor motorbl = null;
    public DcMotor motorintake = null;
    public DcMotorEx motorturret = null;
    //public Limelight3A limelight;

    // Use DcMotorEx for shooter so we can control velocity (ticks/sec)
    public DcMotorEx motorshoot = null;


    // Initial robot orientation
    public YawPitchRollAngles orientation0;
    public AngularVelocity angularVelocity0;
    public double yaw0;

    public robotHardware() {}
    public void init(HardwareMap ahwMap)    {
        hwMap = ahwMap;

        motorfl = hwMap.get(DcMotor.class, "fl");
        motorfr = hwMap.get(DcMotor.class, "fr");
        motorbl = hwMap.get(DcMotor.class, "bl");
        motorbr = hwMap.get(DcMotor.class, "br");
        motorturret = hwMap.get(DcMotorEx.class, "turret");
        motorintake = hwMap.get(DcMotor.class, "intake");

        // get shooter as DcMotorEx to expose velocity control
        motorshoot = hwMap.get(DcMotorEx.class, "shoot");

        //limelight = hwMap.get(Limelight3A.class, "limelight");


        //front right motor no encoder

        motorfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfl.setDirection(DcMotor.Direction.REVERSE);
        motorbl.setDirection(DcMotor.Direction.REVERSE);
        motorintake.setDirection(DcMotor.Direction.REVERSE);


        setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorshoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        orientation0 = imu.getRobotYawPitchRollAngles();
        angularVelocity0 = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        yaw0 = orientation0.getYaw(AngleUnit.DEGREES);
    }

    public void setAutoDriveMotorMode() {
        motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorfr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorfl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getCurrentYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void setDrivetrainMode(DcMotor.RunMode mode) {
        motorfl.setMode(mode);
        motorfr.setMode(mode);
        motorbl.setMode(mode);
        motorbr.setMode(mode);
    }

    public void setDrivePower(double fl, double fr, double bl, double br) {
        if (fl > 1.0)
            fl = 1.0;
        else if (fl < -1.0)
            fl = -1.0;

        if (fr > 1.0)
            fr = 1.0;
        else if (fr < -1.0)
            fr = -1.0;

        if (bl > 1.0)
            bl = 1.0;
        else if (bl < -1.0)
            bl = -1.0;

        if (br > 1.0)
            br = 1.0;
        else if (br < -1.0)
            br = -1.0;

        motorfl.setPower(fl);
        motorfr.setPower(fr);
        motorbl.setPower(bl);
        motorbr.setPower(br);
    }


    // Shooter helpers using DcMotorEx velocity control (ticks/sec)
    public void startShooterAtRPM(double rpm) {
        if (motorshoot == null) return;
        // convert RPM to ticks/sec
        double ticksPerRev = 56; //FIX  2786
        double ticksPerSec = ticksPerRev * rpm / 60.0;
        motorshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorshoot.setVelocity(ticksPerSec);
    }

    public double getShooterRPM() {
        if (motorshoot == null) return 0.0;
        double ticksPerSec = motorshoot.getVelocity();
        double ticksPerRev = motorshoot.getMotorType().getTicksPerRev();
        return ticksPerSec * 60.0 / ticksPerRev;
    }

    public void stopShooter() {
        if (motorshoot == null) return;
        motorshoot.setPower(0.0);
        motorshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void autoShootShort(double rpm){
        startShooterAtRPM(rpm + 50);
        if(motorshoot.getVelocity() > rpm){
            motorintake.setPower(-1.0);
            telemetry.addData("shooter velocity", motorshoot.getVelocity());
            telemetry.addData("shooter tpr", motorshoot.getMotorType().getTicksPerRev());
            telemetry.addData("projected rpm", rpm);
            telemetry.update();
            sleep(400);
            motorintake.setPower(0.0);


        }
    }


}
