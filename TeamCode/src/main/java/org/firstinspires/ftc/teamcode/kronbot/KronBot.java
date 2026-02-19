package org.firstinspires.ftc.teamcode.kronbot;


import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.FLAP_CLOSED;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.kronbot.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.MotorDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.ControlHubGyroscope;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Servo;

/**
 * Container class that holds references for all electronic hardware on the robot
 */
public class KronBot {
    /** Deprecated, is always null. Use PedroPathing follower instead. */
    public MotorDriver motors;
    // Use this instead
    public Follower follower = null;
    public ControlHubGyroscope gyroscope;
    public DcMotorEx intakeMotor, leftOuttake, rightOuttake, loaderMotor;
    public Servo loaderServo, turretServo, angleServo, flapsServo;


    /*
    public void initDrivetrain(HardwareMap hardwareMap) {
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new MotorDriver();
        motors.init(leftRear, leftFront, rightRear, rightFront);
    }
    */

    public void initMotors(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        loaderMotor = hardwareMap.get(DcMotorEx.class, "loaderMotor");

        leftOuttake = hardwareMap.get(DcMotorEx.class, "shooter0");
        leftOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightOuttake = hardwareMap.get(DcMotorEx.class, "shooter1");
        rightOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightOuttake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void initServos(HardwareMap hardwareMap) {
        loaderServo = new Servo(hardwareMap);
        loaderServo.init("loader", true, true, 0, 0, 0);
        loaderServo.runContinuous(false, false);
        turretServo = new Servo(hardwareMap);
        turretServo.init("turretPivot", false, false, 0, 1, 0.5);
        flapsServo = new Servo(hardwareMap);
        flapsServo.init("flapsServo", false, false, 0, 1, FLAP_CLOSED);
        angleServo = new Servo(hardwareMap);
        angleServo.init("anglePivot", false, false, 0, 1, 0);
    }

    public void initSensors(HardwareMap hardwareMap) {
        //outtakeColor = hardwareMap.get(ColorSensor.class, "outtakeColor");
    }

    /**
     * Initialize PedroPathing follower
     * @param hardwareMap Used to initialize drivetrain and localizers
     * @param loadPose If true, will try to load a pose from the file system (if it fails, it loads a 0 pose)
     */
    public void initFollower(HardwareMap hardwareMap, boolean loadPose) {
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        if(loadPose) {
            Pose startingPose = PoseStorage.loadPose();
            follower.setStartingPose(startingPose);
        }
        else
            follower.setStartingPose(new Pose());
    }
    /**
     * Initialize PedroPathing follower with a 0 starting pose
     * @param hardwareMap Used to initialize drivetrain and localizers
     */
    public void initFollower(HardwareMap hardwareMap) { initFollower(hardwareMap, false); }

    /**
     * Initialize PedroPathing follower with a starting pose
     * @param hardwareMap Used to initialize drivetrain and localizers
     * @param startingPose The starting pose
     */
    public void initFollower(HardwareMap hardwareMap, Pose startingPose) {
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
    }


    public void initHardware(HardwareMap hardwareMap) {
        if(follower != null)
            initFollower(hardwareMap);
        initMotors(hardwareMap);
        initServos(hardwareMap);
        initSensors(hardwareMap);
    }
}