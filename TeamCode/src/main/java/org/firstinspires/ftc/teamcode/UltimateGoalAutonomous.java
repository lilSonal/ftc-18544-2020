package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "UltimateGoalAutonomous", group = "auto")

public class UltimateGoalAutonomous extends LinearOpMode {

    Robot robot = new Robot();
    int state = 1;

    //variables
    double drivePower = -0.5;
    double strafePower = -0.5;

    double stackVerticalPosition = 25;
    double stackHorizontalPosition = -16;
    //December 20 (old wheels) Offset: 13.125, 13.125, 23.625, 23.625, 29.125. Mean: 20.525, SD:
    //December 27 (new wheels) Offset: -3.5, -6, -5,75, -5, -5

    double ringNumber = 0;
    double outOfWayPosition = -36;

    double launchingPosition = 23;
    double launchingWheelPower = 0.52;
    double armUpPosition;
    double armDownPosition;

    double bottomZoneVerticalPosition = 10;
    double bottomAndTopZoneHorizontalPosition = 22;
    double middleZoneVerticalPosition = 35;
    double topZoneVerticalPosition = 56;

    double wobbleGoalMotorPower = 0.5;
    double wobbleGoalMotorPosition = 2;
    double wobbleGoalServoPosition = 0.65;


    double middleZoneParkPosition = 23;
    double topZoneParkPosition = 45;

    //cases
    static final int DRIVE_FORWARD_TO_STACK = 1;
    static final int STRAFE_LEFT_TO_STACK = 2;
    static final int CALCULATE_NUMBER_OF_RINGS = 3;
    static final int STRAFE_OUT_OF_WAY = 4;
    static final int DRIVE_FORWARD_TO_LAUNCHING_POSITION = 5;
    static final int LAUNCH_THREE_RINGS = 6;
    static final int DRIVE_TO_CORRECT_ZONE = 7;
    static final int MOVE_ARM = 8;
    static final int DEPOSIT_WOBBLE_GOAL = 9;
    static final int REVERSE_TO_PARK = 10;
    static final int STOP = 11;
    static final int END_STATE = 12;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Begin");
        robot.initForRunToPosition(hardwareMap);
        telemetry.addLine("Initialized!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            //loop()
            /*telemetry.addData("Current State", state);
            telemetry.update();*/
            telemetry.addData("Left front position:", robot.leftFrontMotor.getTargetPosition());
            telemetry.addData("Right front position:", robot.rightFrontMotor.getTargetPosition());
            telemetry.addData("Left back position:", robot.leftBackMotor.getTargetPosition());
            telemetry.addData("Right back position:", robot.rightBackMotor.getTargetPosition());
            telemetry.update();

            switch (state) {

                case (DRIVE_FORWARD_TO_STACK):
                    if (robot.drive(drivePower, stackVerticalPosition)) {
                        goToNextState();
                    }

                    break;

                case (STRAFE_LEFT_TO_STACK):
                    if (robot.strafe(strafePower, stackHorizontalPosition)) {
                        goToState(STRAFE_OUT_OF_WAY);
                        //goToNextState();
                    }
                    break;

                case (CALCULATE_NUMBER_OF_RINGS):
                    //placeholder
                    if (true) {
                        goToNextState();
                    }

                case (STRAFE_OUT_OF_WAY):
                    if (robot.strafe(strafePower, outOfWayPosition)) {
                        goToNextState();
                    }



                case (DRIVE_FORWARD_TO_LAUNCHING_POSITION):
                    if (robot.drive(drivePower, launchingPosition)) {
                        //goToNextState();
                        goToState(11);
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
                    break;

                case (DRIVE_TO_CORRECT_ZONE):
                    if (ringNumber == 0) {
                        if (robot.drive(drivePower, bottomZoneVerticalPosition)) {
                            if (robot.strafe(strafePower, bottomAndTopZoneHorizontalPosition)) {
                                goToNextState();
                            }
                            break;
                        }

                    }
                    else if (ringNumber == 1) {
                        if (robot.drive(drivePower, middleZoneVerticalPosition)) {
                            goToNextState();
                        }
                        break;
                    }
                    else if (ringNumber == 4) {
                        if (robot.drive(drivePower, topZoneVerticalPosition)) {
                            if (robot.strafe(strafePower, bottomAndTopZoneHorizontalPosition)) {
                                goToNextState();
                            }
                            break;
                        }

                    }

                case (MOVE_ARM):
                    robot.moveArm(wobbleGoalMotorPower);
                    goToNextState();
                    break;

                case (DEPOSIT_WOBBLE_GOAL):
                    robot.wobbleGoalServo.setPosition(wobbleGoalServoPosition);
                    goToNextState();
                    break;

                case (REVERSE_TO_PARK):
                    if (ringNumber == 0) {
                        if (robot.drive(drivePower, topZoneParkPosition)) {
                            goToNextState();
                        }
                    }

                    else if (ringNumber == 1) {
                        if (robot.drive(drivePower, middleZoneParkPosition)) {
                            goToNextState();
                        }
                    }
                    break;*/

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
