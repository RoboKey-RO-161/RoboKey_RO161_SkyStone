/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="MoveBase Pod Dreapta", group="Pushbot")

public class MovebaseDreapta extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();   // Use a hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = 0.6;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.motorFataDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFataStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorSpateDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorSpateStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFataDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFataStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorSpateDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorSpateStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                robot.motorFataDreapta.getCurrentPosition(),
                robot.motorFataStanga.getCurrentPosition(),
                robot.motorSpateDreapta.getCurrentPosition(),
                robot.motorSpateStanga.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)


        robot.trapa1.setPosition(0.05);
        robot.trapa2.setPosition(0.95);

        sleep(6500);

        FrontDrive(0.5, -20, -20, -20, -20, 0.9);  // S1: Forward 47 Inches with 5 Sec timeout

        LateralDrive(0.4, 0.4, -10, 10, -10, 10, 1.6);

        robot.movebase1.setPosition(1.0);
        robot.movebase2.setPosition(0.0);
        sleep(1500);

        LateralDrive(0.5, 0.7, 10, -10, 10, -10, 1.3);
        TurnDrive(0.2, 0.4, 10, 10, 10, 10, 2.4);

        sleep(200);
        robot.movebase1.setPosition(0.0);
        robot.movebase2.setPosition(1.0);

        LateralDrive(0.5, 0.5, -20, 20, -20, 20, 2.0);
        sleep(1000);
        LateralDrive(0.5, 0.5, 20, -20, 20, -20, 0.3);
        sleep(100);

        TurnDrive(0.4, 0.4, 10, -10, -10, 10, 0.45);
        sleep(100);
        LateralDrive(0.4, 0.4, 10, -10, 10, -10, 1.3);
        sleep(400);

        FrontDrive(DRIVE_SPEED, 20, 20, 20, 20, 1.0);  // S1: Forward 47 Inches with 5 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void FrontDrive(double speed, double FataDreapta, double FataStanga, double SpateDreapta, double SpateStanga, double timeoutS) {
        int newLeftFTarget;
        int newRightFTarget;
        int newLeftRTarget;
        int newRightRTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFTarget = robot.motorFataDreapta.getCurrentPosition() + (int) (FataDreapta * COUNTS_PER_INCH);
            newRightFTarget = robot.motorFataStanga.getCurrentPosition() + (int) (FataStanga * COUNTS_PER_INCH);
            newRightRTarget = robot.motorSpateDreapta.getCurrentPosition() + (int) (SpateDreapta * COUNTS_PER_INCH);
            newLeftRTarget = robot.motorSpateStanga.getCurrentPosition() + (int) (SpateStanga * COUNTS_PER_INCH);

            robot.motorFataDreapta.setTargetPosition(newRightFTarget);
            robot.motorFataStanga.setTargetPosition(newLeftFTarget);
            robot.motorSpateDreapta.setTargetPosition(newRightRTarget);
            robot.motorSpateStanga.setTargetPosition(newLeftRTarget);


            // Turn On RUN_TO_POSITION
            robot.motorFataDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFataStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorFataDreapta.setPower(Math.abs(speed));
            robot.motorFataStanga.setPower(Math.abs(speed));
            robot.motorSpateDreapta.setPower(Math.abs(speed));
            robot.motorSpateStanga.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorFataDreapta.isBusy() && robot.motorFataStanga.isBusy() && robot.motorSpateDreapta.isBusy() && robot.motorSpateStanga.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newRightFTarget, newLeftFTarget, newRightRTarget, newLeftRTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        robot.motorFataDreapta.getCurrentPosition(),
                        robot.motorFataStanga.getCurrentPosition(),
                        robot.motorSpateDreapta.getCurrentPosition(),
                        robot.motorSpateStanga.getCurrentPosition()
                );
                telemetry.update();
            }

            // Stop all motion;
            robot.motorFataDreapta.setPower(0);
            robot.motorFataStanga.setPower(0);
            robot.motorSpateDreapta.setPower(0);
            robot.motorSpateStanga.setPower(0);

            robot.motorFataDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFataStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION
            robot.motorFataDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFataStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(400);   // optional pause after each move
        }
    }

    public void LateralDrive(double speed1, double speed2, double FataDreapta, double FataStanga, double SpateDreapta, double SpateStanga, double timeoutS) {

        int newLeftFTarget;
        int newRightFTarget;
        int newLeftRTarget;
        int newRightRTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFTarget = robot.motorFataDreapta.getCurrentPosition() + (int) (FataDreapta * COUNTS_PER_INCH);
            newRightFTarget = robot.motorFataStanga.getCurrentPosition() + (int) (FataStanga * COUNTS_PER_INCH);
            newRightRTarget = robot.motorSpateDreapta.getCurrentPosition() + (int) (SpateDreapta * COUNTS_PER_INCH);
            newLeftRTarget = robot.motorSpateStanga.getCurrentPosition() + (int) (SpateStanga * COUNTS_PER_INCH);

            robot.motorFataDreapta.setTargetPosition(newRightFTarget);
            robot.motorFataStanga.setTargetPosition(newLeftFTarget);
            robot.motorSpateDreapta.setTargetPosition(newRightRTarget);
            robot.motorSpateStanga.setTargetPosition(newLeftRTarget);


            // Turn On RUN_TO_POSITION
            robot.motorFataDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFataStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorFataDreapta.setPower(Math.abs(speed1));
            robot.motorFataStanga.setPower(Math.abs(speed1));
            robot.motorSpateDreapta.setPower(Math.abs(speed2));
            robot.motorSpateStanga.setPower(Math.abs(speed2));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorFataDreapta.isBusy() && robot.motorFataStanga.isBusy() && robot.motorSpateDreapta.isBusy() && robot.motorSpateStanga.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newRightFTarget, newLeftFTarget, newRightRTarget, newLeftRTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        robot.motorFataDreapta.getCurrentPosition(),
                        robot.motorFataStanga.getCurrentPosition(),
                        robot.motorSpateDreapta.getCurrentPosition(),
                        robot.motorSpateStanga.getCurrentPosition()
                );
                telemetry.update();
            }

            // Stop all motion;
            robot.motorFataDreapta.setPower(0);
            robot.motorFataStanga.setPower(0);
            robot.motorSpateDreapta.setPower(0);
            robot.motorSpateStanga.setPower(0);


            robot.motorFataDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFataStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION
            robot.motorFataDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFataStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public void TurnDrive(double speed1, double speed2, double FataDreapta, double FataStanga, double SpateDreapta, double SpateStanga, double timeoutS) {
        int newLeftFTarget;
        int newRightFTarget;
        int newLeftRTarget;
        int newRightRTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFTarget = robot.motorFataDreapta.getCurrentPosition() + (int) (FataDreapta * COUNTS_PER_INCH);
            newRightFTarget = robot.motorFataStanga.getCurrentPosition() + (int) (FataStanga * COUNTS_PER_INCH);
            newRightRTarget = robot.motorSpateDreapta.getCurrentPosition() + (int) (SpateDreapta * COUNTS_PER_INCH);
            newLeftRTarget = robot.motorSpateStanga.getCurrentPosition() + (int) (SpateStanga * COUNTS_PER_INCH);

            robot.motorFataDreapta.setTargetPosition(newRightFTarget);
            robot.motorFataStanga.setTargetPosition(newLeftFTarget);
            robot.motorSpateDreapta.setTargetPosition(newRightRTarget);
            robot.motorSpateStanga.setTargetPosition(newLeftRTarget);


            // Turn On RUN_TO_POSITION
            robot.motorFataDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFataStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorFataDreapta.setPower(Math.abs(speed1));
            robot.motorFataStanga.setPower(Math.abs(speed1));
            robot.motorSpateDreapta.setPower(Math.abs(speed2));
            robot.motorSpateStanga.setPower(Math.abs(speed2));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorFataDreapta.isBusy() && robot.motorFataStanga.isBusy() && robot.motorSpateDreapta.isBusy() && robot.motorSpateStanga.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newRightFTarget, newLeftFTarget, newRightRTarget, newLeftRTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        robot.motorFataDreapta.getCurrentPosition(),
                        robot.motorFataStanga.getCurrentPosition(),
                        robot.motorSpateDreapta.getCurrentPosition(),
                        robot.motorSpateStanga.getCurrentPosition()
                );
                telemetry.update();
            }

            // Stop all motion;
            robot.motorFataDreapta.setPower(0);
            robot.motorFataStanga.setPower(0);
            robot.motorSpateDreapta.setPower(0);
            robot.motorSpateStanga.setPower(0);


            robot.motorFataDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFataStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION
            robot.motorFataDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFataStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
}