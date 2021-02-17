package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "UltimateGoalAutonomous", group = "auto")

public class UltimateGoalAutonomous extends LinearOpMode {

    Robot robot = new Robot();
    int state = 1;

    //variables
    double drivePower = -0.5;
    double strafePower = -0.5;

    //double stackVerticalPosition = 25;
    //double stackHorizontalPosition = -16;
    double stackVerticalPosition = 39;
    double stackHorizontalPosition = -5;
    //December 20 (old wheels) Offset: 13.125, 13.125, 23.625, 23.625, 29.125. Mean: 20.525, SD:
    //December 27 (new wheels) Offset: -3.5, -6, -5,75, -5, -5

    double oneRingThreshold = 292.5; //285 earlier, then 290
    double fourRingThreshold = 272.5; //235 earlier, then 270, then 275
    double ringNumber = 0;
    double outOfWayPosition = -(stackHorizontalPosition);

    double launchingPosition = 26; //23 before
    double launchingWheelPower = 0.52;
    double armUpPosition;
    double armDownPosition;

    double bottomZoneVerticalPosition = -9; //10 before
    double bottomAndTopZoneHorizontalPosition = 0; //25 before
    double middleZoneHorizontalPosition = -15; //-12 before
    double middleZoneVerticalPosition = 8; //23 before, then 10, then -12
    double topZoneVerticalPosition = 36.5; //44 before

    double wobbleGoalMotorPower = 0.20; //0.25 before
    int wobbleGoalDownPosition = 300; //470 before, then 400, then 350, then 300
    int wobbleGoalUpPosition = 300; //30 before, then 0, then 100
    double wobbleGoalOpenPosition = 0.65; //0.85 before, 1
    double wobbleGoalClosePosition = 0.9; //0.65 before, 0.35
    int sleepTime = 4000; //500 before, then 2000

    double bottomZoneParkPosition = 16; //20 before, then 12
    double bottomZoneParkStrafe = -20;
    double middleZoneParkPosition = 0; //23 before
    double topZoneParkPosition = -30; //-45 before

    //cases
    static final int DRIVE_FORWARD_TO_STACK = 1;
    static final int STRAFE_LEFT_TO_STACK = 2;
    static final int CALCULATE_NUMBER_OF_RINGS = 3;
    static final int STRAFE_OUT_OF_WAY = 4;
    static final int DRIVE_FORWARD_TO_LAUNCHING_POSITION = 5;
    static final int LAUNCH_THREE_RINGS = 6;
    static final int DRIVE_TO_CORRECT_ZONE = 7;
    static final int STRAFE_TO_CORRECT_ZONE = 8;
    static final int MOVE_ARM_DOWN = 9;
    static final int DEPOSIT_WOBBLE_GOAL = 10;
    static final int CLOSE_ARM = 11;
    static final int RETRACT_ARM = 12;
    static final int STRAFE_TO_PARK = 13;
    static final int REVERSE_TO_PARK = 14;
    static final int STOP = 15;
    static final int END_STATE = 16;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initForRunToPosition(hardwareMap);
        /*telemetry.addLine("Begin");
        telemetry.addLine("Initialized!");
        telemetry.update();*/
        waitForStart();

        while (opModeIsActive()) {
            robot.wobbleGoalServo.setPosition(wobbleGoalClosePosition);
            //loop()
            /*telemetry.addData("Current State", state);
            telemetry.update();*/
            /*telemetry.addData("Left front position:", robot.leftFrontMotor.getTargetPosition());
            telemetry.addData("Right front position:", robot.rightFrontMotor.getTargetPosition());
            telemetry.addData("Left back position:", robot.leftBackMotor.getTargetPosition());
            telemetry.addData("Right back position:", robot.rightBackMotor.getTargetPosition());*/
            telemetry.addData("State: ", state);

            //telemetry.addData("deviceName", robot.sensorRange.getDeviceName());
            int ringTelemetryNumber = 0;
            telemetry.addData("Range", String.format("%.01f mm", robot.sensorRange.getDistance(DistanceUnit.MM)));
            if (robot.sensorRange.getDistance(DistanceUnit.MM) <= fourRingThreshold) {
                ringTelemetryNumber = 4;
            }
            else if (robot.sensorRange.getDistance(DistanceUnit.MM) <= oneRingThreshold) { //285 earlier
                ringTelemetryNumber = 1;
            }
            else if (robot.sensorRange.getDistance(DistanceUnit.MM) > oneRingThreshold) { //285 earlier
                ringTelemetryNumber = 0;
            }
            telemetry.addData("Rings", ringTelemetryNumber);
            //telemetry.update();
            telemetry.update();

            switch (state) {

                case (DRIVE_FORWARD_TO_STACK):
                    /*if (true) {
                        goToState(11);
                    }*/
                    if (robot.drive(drivePower, stackVerticalPosition)) {
                        goToNextState();
                    }

                    break;

                case (STRAFE_LEFT_TO_STACK):
                    if (robot.strafe(strafePower, stackHorizontalPosition)) {
                        //goToState(DRIVE_FORWARD_TO_LAUNCHING_POSITION);
                        goToNextState();
                    }
                    break;

                case (CALCULATE_NUMBER_OF_RINGS):

                    //placeholder
                    /*if (true) {
                        goToNextState();
                    }*/
                    //calculation attempt
                    if (robot.sensorRange.getDistance(DistanceUnit.MM) <= fourRingThreshold) {
                        ringNumber = 4;
                    }
                    else if (robot.sensorRange.getDistance(DistanceUnit.MM) <= oneRingThreshold) {
                        ringNumber = 1;
                    }
                    else if (robot.sensorRange.getDistance(DistanceUnit.MM) > oneRingThreshold) {
                        ringNumber = 0;
                    }
                    telemetry.addData("Range", String.format("%.01f mm", robot.sensorRange.getDistance(DistanceUnit.MM)));
                    telemetry.addData("Rings calculated: ", ringNumber);
                    telemetry.update();
                    //goToState(DRIVE_FORWARD_TO_LAUNCHING_POSITION);
                    //goToState(12);
                    goToNextState();
                    break;

                case (STRAFE_OUT_OF_WAY):
                    if (robot.strafe(strafePower, outOfWayPosition)) {
                        //goToNextState();
                        goToNextState();
                    }
                    break;



                case (DRIVE_FORWARD_TO_LAUNCHING_POSITION):
                    if (robot.drive(drivePower, launchingPosition)) {
                        //goToNextState();
                        //goToState(12);
                        goToState(DRIVE_TO_CORRECT_ZONE);
                    }
                    break;

                /*case (LAUNCH_THREE_RINGS):
                    robot.launchingWheel.setPower(launchingWheelPower);
                    for (int i=1; i<=3; i++) {
                        //robot.launchingWheel.setPower(launchingWheelPower);
                        robot.launchingServo.setPosition(LAUNCHING_SERVO_ACTIVE_POSITION);
                        robot.launchingServo.setPosition(LAUNCHING_SERVO_INACTIVE_POSITION);

                    }
                    robot.launchingWheel.setPower(0);
                    goToNextState();
                    break;*/

                /*case (DRIVE_TO_CORRECT_ZONE):
                    if (ringNumber == 0) {
                        telemetry.addData("Path A for zero rings", state);
                        telemetry.update();

                        if (robot.drive(drivePower, bottomZoneVerticalPosition)) {
                            if (robot.strafe(strafePower, bottomAndTopZoneHorizontalPosition)) {
                                //goToNextState();
                                goToState(11);
                            }
                            break;
                        }

                    }
                    else if (ringNumber == 1) {
                        telemetry.addData("Path B for one ring", state);
                        telemetry.update();
                        if (robot.drive(drivePower, middleZoneVerticalPosition)) {
                            //goToNextState();
                            goToState(11);
                        }
                        break;
                    }
                    else if (ringNumber == 4) {
                        telemetry.addData("Path C for four rings", state);
                        telemetry.update();
                        if (robot.drive(drivePower, topZoneVerticalPosition)) {
                            if (robot.strafe(strafePower, bottomAndTopZoneHorizontalPosition)) {
                                //goToNextState();
                                goToState(11);
                            }
                            break;
                        }

                    }*/
                case (DRIVE_TO_CORRECT_ZONE):
                    if (ringNumber == 0) {
                        telemetry.addData("Path A for zero rings", state);
                        telemetry.update();

                        if (robot.drive(drivePower, bottomZoneVerticalPosition)) {
                            goToNextState();

                        }
                        break;

                    }
                    else if (ringNumber == 1) {
                        telemetry.addData("Path B for one ring", state);
                        telemetry.update();
                        if (robot.drive(drivePower, middleZoneVerticalPosition)) {
                            goToNextState();

                        }
                        break;
                    }
                    else if (ringNumber == 4) {
                        telemetry.addData("Path C for four rings", state);
                        telemetry.update();
                        if (robot.drive(drivePower, topZoneVerticalPosition)) {
                            goToNextState();

                        }
                        break;

                    }
                case (STRAFE_TO_CORRECT_ZONE):
                    if (ringNumber == 0) {
                        telemetry.addData("Path A for zero rings", state);
                        telemetry.update();

                        if (robot.strafe(strafePower, bottomAndTopZoneHorizontalPosition)) {
                            //goToState(STRAFE_TO_PARK);
                            //goToState(STOP);
                            goToNextState();
                        }
                        break;

                    }
                    else if (ringNumber == 1) {
                        telemetry.addData("Path B for one ring", state);
                        telemetry.update();
                        if (robot.strafe(strafePower, middleZoneHorizontalPosition)) {
                            //goToState(STRAFE_TO_PARK);
                            //goToState(STOP);
                            goToNextState();
                        }
                        break;
                    }
                    else if (ringNumber == 4) {
                        telemetry.addData("Path C for four rings", state);
                        telemetry.update();
                        if (robot.strafe(strafePower, bottomAndTopZoneHorizontalPosition)) {
                            //goToState(STRAFE_TO_PARK);
                            //goToState(STOP);
                            goToNextState();
                        }
                        break;

                    }



                case (MOVE_ARM_DOWN):
                    //robot.moveArm(wobbleGoalMotorPower);
                    if (robot.moveArm(wobbleGoalDownPosition, wobbleGoalMotorPower)) {
                        goToNextState();
                    }

                    break;

                case (DEPOSIT_WOBBLE_GOAL):
                    robot.wobbleGoalServo.setPosition(wobbleGoalOpenPosition);
                    sleep(sleepTime);
//                    robot.moveArmServo(0.85))
                    goToNextState();

                    break;

                case (CLOSE_ARM):
                    //robot.wobbleGoalServo.setPosition(wobbleGoalServoPosition);
                    /*if (robot.moveArmServo(0.65)) {
                        goToNextState();
                    }*/

                    //robot.wobbleGoalServo.setPosition(wobbleGoalClosePosition);
                    //sleep(sleepTime);

                    goToNextState();
                    //goToState(STOP);

                    break;

                case (RETRACT_ARM):
                    //robot.wobbleGoalServo.setPosition(wobbleGoalServoPosition);
                    if (robot.moveArm(wobbleGoalUpPosition, wobbleGoalMotorPower)) {
                        goToNextState();
                    }

                    break;

                case (STRAFE_TO_PARK):

                    if (ringNumber == 0) {
                        if (robot.strafe(strafePower, bottomZoneParkStrafe)) {
                            goToNextState();
                            break;
                        }
                    }

                    else {
                        goToNextState();
                        break;
                    }
                    break;





                case (REVERSE_TO_PARK):
                    if (ringNumber == 4) {
                        if (robot.drive(drivePower, topZoneParkPosition)) {
                            goToNextState();
                            break;
                        }
                    }


                    else if (ringNumber == 1) {
                        if (robot.drive(drivePower, middleZoneParkPosition)) {
                            goToNextState();
                            break;
                        }
                    }


                    else if (ringNumber == 0) {
                        if (robot.drive(drivePower, bottomZoneParkPosition)) {
                            goToNextState();
                            break;
                        }
                    }
                    break;

                case (STOP):
                    robot.stop();
                    goToNextState();
                    break;

                default:
                    state = END_STATE;
                    break;

            }
        }
    }

    public void goToNextState() {
        state++;
    }

    public void goToState(int newState) {
        state = newState;
    }



}
