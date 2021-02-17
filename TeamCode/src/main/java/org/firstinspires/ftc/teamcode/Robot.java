package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

/* This is a class defining all of the constants, limits, initial positions, methods, motors, servos, switches, and sensors that are used in
    autonomous and teleop.
    */
public class Robot {
    public boolean encodersReseted = false;

    Timer timer = new Timer();
    // Wheels
    double wheelDiameter = 4;
    double wheelInchesPerRotation = Math.PI * wheelDiameter;
    int motorTicksPerRotation = 1120;
    double gearRatioMotorToWheel = 2.0/1.0;
    // double type for higher accuracy when multiplying by distanceInch in driveForward() method
    double robotTicksPerInch = motorTicksPerRotation / (gearRatioMotorToWheel * wheelInchesPerRotation);

    // Chassis motors
    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;

    //Other motors
    public DcMotorEx launchingWheel;
    public DcMotorEx wobbleGoalMotor;
    public DcMotorEx collectorMotor;

    //Servos
    public Servo launchingServo;
    public Servo wobbleGoalServo;
    public Servo deliveryLiftServo;
    public Servo deliveryRingServo;

    //Sensors
    public DistanceSensor sensorRange;

    //---Constants---//
    public static final int LAUNCHING_SERVO_ACTIVE_POSITION = 0;
    public static final int LAUNCHING_SERVO_INACTIVE_POSITION = 0;
    public static final int MOTOR_ENCODER_TOLERANCE = 50;
    public static final int WOBBLE_GOAL_MOTOR_TICKS_PER_ROTATION = 7168;
    public static final int WOBBLE_GOAL_INITIAL_ANGLE_STARTING_DIFFERENCE_FROM_0_DEG = 1;
    public static final int WOBBLE_GOAL_MOTOR_UP_LIMIT = 1400;
    public static final int WOBBLE_GOAL_MOTOR_DOWN_LIMIT = 0;


    public void initForRunToPosition(HardwareMap hardwareMap) {
        this.init(hardwareMap);
        setModeChassisMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void initRegular(HardwareMap hardwareMap) {
        this.init(hardwareMap);
        setModeChassisMotors(RUN_USING_ENCODER);
    }

    public boolean drive(double power, double distanceInch) {
        if(!encodersReseted) {
            this.resetChassisEncoders();
            encodersReseted = true;
        }
        // Getting the sign of the argument to determine which direction we're driving
        int direction = (int)Math.signum(distanceInch);

        this.leftFrontMotor.setTargetPosition((int)(distanceInch * robotTicksPerInch));
        this.rightFrontMotor.setTargetPosition((int)(distanceInch * robotTicksPerInch));
        this.leftBackMotor.setTargetPosition((int)(distanceInch * robotTicksPerInch));
        this.rightBackMotor.setTargetPosition((int)(distanceInch * robotTicksPerInch));

        this.leftFrontMotor.setPower(direction * power);
        this.rightFrontMotor.setPower(direction * power);
        this.leftBackMotor.setPower(direction * power);
        this.rightBackMotor.setPower(direction * power);

        setModeChassisMotors(DcMotor.RunMode.RUN_TO_POSITION);
        if (!this.leftFrontMotor.isBusy() || !this.leftBackMotor.isBusy() || !this.rightFrontMotor.isBusy() || !this.rightBackMotor.isBusy()) {
            encodersReseted = false;
            return true;
        } else {
            return false;
        }
    }

    public boolean strafe(double power, double distanceInch) {
        if(!encodersReseted) {
            this.resetChassisEncoders();
            encodersReseted = true;
        }
        // Getting the sign of the argument to determine which direction we're strafing
        int direction = (int)Math.signum(distanceInch);

        this.leftFrontMotor.setTargetPosition((int)(distanceInch * robotTicksPerInch));
        this.rightFrontMotor.setTargetPosition(-(int)(distanceInch * robotTicksPerInch));
        this.leftBackMotor.setTargetPosition(-(int)(distanceInch * robotTicksPerInch));
        this.rightBackMotor.setTargetPosition((int)(distanceInch * robotTicksPerInch));

        this.leftFrontMotor.setPower(direction * power);
        this.rightFrontMotor.setPower(-direction * power);
        this.leftBackMotor.setPower(-direction * power);
        this.rightBackMotor.setPower(direction * power);

        setModeChassisMotors(DcMotor.RunMode.RUN_TO_POSITION);
        /*telemetry.addData("Left front position:", this.leftFrontMotor.getTargetPosition());
        telemetry.addData("Right front position:", this.rightFrontMotor.getTargetPosition());
        telemetry.addData("Left back position:", this.leftBackMotor.getTargetPosition());
        telemetry.addData("Right back position:", this.rightBackMotor.getTargetPosition());*/

        if (!this.leftFrontMotor.isBusy() || !this.leftBackMotor.isBusy() || !this.rightFrontMotor.isBusy() || !this.rightBackMotor.isBusy()) {
            encodersReseted = false;
            return true;
        } else {
            return false;
        }
    }

    private void init(HardwareMap hardwareMap) {

        //chassis
        this.leftFrontMotor = hardwareMap.get(DcMotorEx.class, "LeftFront");
        this.rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RightFront");
        this.leftBackMotor = hardwareMap.get(DcMotorEx.class, "LeftBack");
        this.rightBackMotor = hardwareMap.get(DcMotorEx.class, "RightBack");

        this.rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.leftFrontMotor.setTargetPositionTolerance(MOTOR_ENCODER_TOLERANCE);
        this.rightFrontMotor.setTargetPositionTolerance(MOTOR_ENCODER_TOLERANCE);
        this.leftBackMotor.setTargetPositionTolerance(MOTOR_ENCODER_TOLERANCE);
        this.leftBackMotor.setTargetPositionTolerance(MOTOR_ENCODER_TOLERANCE);

        //wobble goal
        wobbleGoalMotor = hardwareMap.get(DcMotorEx.class, "wobbleGoalMotor");
        wobbleGoalServo = hardwareMap.get(Servo.class, "wobbleGoalServo");

        wobbleGoalMotor.setMode(STOP_AND_RESET_ENCODER);
        wobbleGoalMotor.setTargetPosition(0);
        wobbleGoalMotor.setMode(RUN_TO_POSITION);
        wobbleGoalMotor.setPower(0.2);

        //collector
        collectorMotor = hardwareMap.get(DcMotorEx.class, "collectorMotor");

        //launching wheel
        launchingWheel = hardwareMap.get(DcMotorEx.class, "launchingWheel");
        launchingWheel.setMode(RUN_USING_ENCODER);

        //distance sensor
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;


        //delivery
        deliveryLiftServo = hardwareMap.get(Servo.class, "deliveryLiftServo");
        deliveryRingServo = hardwareMap.get(Servo.class, "deliveryRingServo");
    }

    public void stop() {
        this.leftBackMotor.setPower(0.0);
        this.rightBackMotor.setPower(0.0);
        this.leftFrontMotor.setPower(0.0);
        this.rightFrontMotor.setPower(0.0);
    }

    /*
     * Set the runMode of all the chassis motors
     * @param runMode: the runMode the chassis motors should be set to
     */
    public void setModeChassisMotors(DcMotor.RunMode runMode) {
        this.leftFrontMotor.setMode(runMode);
        this.rightFrontMotor.setMode(runMode);
        this.leftBackMotor.setMode(runMode);
        this.rightBackMotor.setMode(runMode);
    }

    public void resetChassisEncoders() {
        if(this.leftFrontMotor.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            setModeChassisMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    /*
     * Drive method for when the distance to drive is unspecified
     * Before using this method, ensure the motors are in the run mode RUN_USING_ENCODER
     */
    public void drivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower){
        this.leftFrontMotor.setPower(leftFrontPower);
        this.rightFrontMotor.setPower(rightFrontPower);
        this.leftBackMotor.setPower(leftBackPower);
        this.rightBackMotor.setPower(rightBackPower);
    }

    /*public boolean moveArm(double angle){
        int wobbleGoalMotorPosition = (int)(WOBBLE_GOAL_MOTOR_TICKS_PER_ROTATION * (angle + WOBBLE_GOAL_INITIAL_ANGLE_STARTING_DIFFERENCE_FROM_0_DEG) / 360); //Change angle offset
        if (wobbleGoalMotorPosition > WOBBLE_GOAL_MOTOR_UP_LIMIT) wobbleGoalMotorPosition = WOBBLE_GOAL_MOTOR_UP_LIMIT;
        if (wobbleGoalMotorPosition < WOBBLE_GOAL_MOTOR_DOWN_LIMIT) wobbleGoalMotorPosition = WOBBLE_GOAL_MOTOR_DOWN_LIMIT;
        this.wobbleGoalMotor.setTargetPosition(wobbleGoalMotorPosition);
        this.wobbleGoalMotor.setPower(1.0);



        return !this.wobbleGoalMotor.isBusy();
    }*/

    public boolean moveArm(int target, double power) {
        this. wobbleGoalMotor.setTargetPosition(target);
        this.wobbleGoalMotor.setPower(power);

        return !this.wobbleGoalMotor.isBusy();
    }

//    public boolean moveArmServo(double target, ElapsedTime timer) {
//        this. wobbleGoalServo.setPosition(target);
//
//
//        return !this.wobbleGoalServo.isBusy();
//
//    }



}
