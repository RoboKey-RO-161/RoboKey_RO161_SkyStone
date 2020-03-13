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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;


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

@Autonomous(name="Brick", group="Pushbot")

public class Brick extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    double DRIVE_SPEED ;

    double stone = 0.0;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();



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

            robot.trapa1.setPosition(0.6);
            robot.trapa2.setPosition(0.4);

            sleep(1000);

           DetectDrive(20);
           sleep(100);
           BrickDrive(1,1,1,1,10);
           sleep(100);
           Prindere(0.35);
           sleep(100);
           Apropiere(0.4);
           robot.brick.setPosition(1.0);
           sleep(500);
           DRIVE_SPEED = 0.7;
           LateralDrive(1,1,1,1,0.8);
           sleep(100);
           FrontDrive(1,1,1,1,3.3);
           sleep(100);
           robot.brick.setPosition(0.5);
           sleep(100);




           RearDrive(1,1,1,1,1.5);
         /*  sleep(100);
           DetectDrive(15);
           sleep(100);
           BrickDrive(1,1,1,1,10);
           sleep(100);
           Prindere(0.5);
           sleep(100);
           Apropiere(0.5);
           sleep(100);
           robot.brick.setPosition(1.0);
           sleep(500);
           LateralDrive(1,1,1,1,0.9);
           sleep(100);
           FrontDrive(1,1,1,1,3.6);
           sleep(100);
           robot.brick.setPosition(0.0);
           sleep(500);
           RearDrive(1,1,1,1,1.0);
*/

    }


    public void DetectDrive(double timeoutS) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            // reset the timeout time and start motion.
            runtime.reset();
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.distance_sensor.getDistance(DistanceUnit.CM) >= 10.0)
            ) {
                robot.motorFataDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.motorFataStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.motorSpateDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.motorSpateStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if(robot.distance_sensor.getDistance(DistanceUnit.CM) <= 30 ) {
                    robot.motorFataDreapta.setPower(-0.19);
                    robot.motorFataStanga.setPower(0.19);
                    robot.motorSpateDreapta.setPower(0.21);
                    robot.motorSpateStanga.setPower(-0.21);

                }
                // Display it for the driver.
                else if(runtime.seconds() < 1)
                {
                    robot.motorFataDreapta.setPower(-0.3);
                    robot.motorFataStanga.setPower(0.3);
                    robot.motorSpateDreapta.setPower(0.3);
                    robot.motorSpateStanga.setPower(-0.3);

                }
               else {
                    robot.motorFataDreapta.setPower(-0.5);
                    robot.motorFataStanga.setPower(0.5);
                    robot.motorSpateDreapta.setPower(0.52);
                    robot.motorSpateStanga.setPower(-0.52);
                }
                NormalizedRGBA colors = robot.color_sensor.getNormalizedColors();
                telemetry.addData("Alpha %3f",colors.alpha );
                telemetry.addData("Red %3f",colors.red );
                telemetry.addData("Green %3f",colors.green );
                telemetry.addData("Blue %3f",colors.blue );
                telemetry.addLine(" ");
                telemetry.addData("Device:", robot.distance_sensor.getDeviceName());
                telemetry.addData("range:", String.format("%.01f cm", robot.distance_sensor.getDistance(DistanceUnit.CM)));
                telemetry.update();

            }


            // Stop all motion;
            robot.motorFataDreapta.setPower(0);
            robot.motorFataStanga.setPower(0);
            robot.motorSpateDreapta.setPower(0);
            robot.motorSpateStanga.setPower(0);

            robot.motorFataDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorFataStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorSpateDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorSpateStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




            NormalizedRGBA colors = robot.color_sensor.getNormalizedColors();


            telemetry.addData("Alpha :%3f", colors.alpha);
            telemetry.addData("Red :%3f", colors.red);
            telemetry.addData("Green :%3f", colors.green);
            telemetry.addData("Blue :%3f", colors.blue);
            telemetry.addLine("       ");

            telemetry.update();


            if((colors.alpha >= 0.0000 && colors.alpha <= 0.0059)&&
                        (colors.red >=0.0000 && colors.red <= 0.0016)&&
                        (colors.green >=0.0000 && colors.green <= 0.0020)&&
                        (colors.blue >=0.0000&& colors.blue <= 0.0019)) {
                telemetry.addData("Skystone", "Ok");
                stone=1;
            }

            else {
                stone = 0;
                telemetry.addData("SkyStone","No");
            }

            telemetry.update();

            sleep(200);
            robot.motorFataDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.motorFataStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.motorSpateDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.motorSpateStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            sleep(250);   // optional pause after each move
        }
    }

    public void BrickDrive(double FataDreapta,double FataStanga,double SpateDreapta,double SpateStanga, double timeoutS){
        int newLeftFTarget;
        int newRightFTarget;
        int newLeftRTarget;
        int newRightRTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            NormalizedRGBA colors = robot.color_sensor.getNormalizedColors();
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

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    !(colors.alpha >= 0.0000 && colors.alpha <= 0.0060)&&
                    !(colors.red >=0.0000 && colors.red <= 0.00159)&&
                    !(colors.green >=0.0000 && colors.green <= 0.00196)&&
                    !(colors.blue >=0.0000&& colors.blue <= 0.0018)
            ) {
                robot.motorFataDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.motorFataStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.motorSpateDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.motorSpateStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.motorFataDreapta.setPower(-0.25);
                robot.motorFataStanga.setPower(-0.19);
                robot.motorSpateDreapta.setPower(-0.23);
                robot.motorSpateStanga.setPower(-0.23);

                colors = robot.color_sensor.getNormalizedColors();

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newRightFTarget, newLeftFTarget, newRightRTarget, newLeftRTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d", robot.motorFataDreapta.getCurrentPosition(), robot.motorFataStanga.getCurrentPosition(), robot.motorSpateDreapta.getCurrentPosition(), robot.motorSpateStanga.getCurrentPosition());
                telemetry.addLine(" ");
                telemetry.addData("Alpha %3f",colors.alpha );
                telemetry.addData("Red %3f",colors.red );
                telemetry.addData("Green %3f",colors.green );
                telemetry.addData("Blue %3f",colors.blue );
                telemetry.addLine(" ");
                telemetry.addData("Device:", robot.distance_sensor.getDeviceName());
                telemetry.addData("range:", String.format("%.01f cm", robot.distance_sensor.getDistance(DistanceUnit.CM)));
                telemetry.update();
                sleep(20);
            }




            // Stop all motion;
            robot.motorFataDreapta.setPower(0);
            robot.motorFataStanga.setPower(0);
            robot.motorSpateDreapta.setPower(0);
            robot.motorSpateStanga.setPower(0);

            robot.motorFataDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorFataStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorSpateDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorSpateStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




            // NormalizedRGBA colors = robot.color_sensor.getNormalizedColors();


            telemetry.addData("Alpha :%3f", colors.alpha);
            telemetry.addData("Red :%3f", colors.red);
            telemetry.addData("Green :%3f", colors.green);
            telemetry.addData("Blue :%3f", colors.blue);
            telemetry.addLine("       ");

            telemetry.update();



            telemetry.addData("Skystone", "Ok");
            stone=1;



            telemetry.update();

            sleep(200);
            robot.motorFataDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFataStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION
            robot.motorFataDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFataStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.motorFataDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.motorFataStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.motorSpateDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.motorSpateStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            sleep(200);   // optional pause after each move
        }
    }

    public void LateralDrive(double FataDreapta,double FataStanga,double SpateDreapta,double SpateStanga, double timeoutS) {
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
            robot.motorFataDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorFataStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();

           // while (opModeIsActive() &&
          //          (runtime.seconds() < timeoutS)
            if (runtime.seconds() >= timeoutS-((10/100)*timeoutS)){
                robot.motorFataDreapta.setPower(0.3);
                robot.motorFataStanga.setPower(-0.3);
                robot.motorSpateDreapta.setPower(-0.3);
                robot.motorSpateStanga.setPower(0.3);
            }   else {
                robot.motorFataDreapta.setPower(DRIVE_SPEED);
                robot.motorFataStanga.setPower(-DRIVE_SPEED);
                robot.motorSpateDreapta.setPower(-DRIVE_SPEED);
                robot.motorSpateStanga.setPower(DRIVE_SPEED);
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
            ) {

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

            robot.motorFataDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorFataStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorSpateDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorSpateStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            robot.motorFataDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFataStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION

            sleep(200);   // optional pause after each move
        }
    }

    public void FrontDrive(double FataDreapta,double FataStanga,double SpateDreapta,double SpateStanga, double timeoutS) {
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
            robot.motorFataDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorFataStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // reset the timeout time and start motion.

                robot.motorFataDreapta.setPower(DRIVE_SPEED);
                robot.motorFataStanga.setPower(DRIVE_SPEED);
                robot.motorSpateDreapta.setPower(DRIVE_SPEED);
                robot.motorSpateStanga.setPower(DRIVE_SPEED);
                runtime.reset();
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
            ) {

                if (runtime.seconds() >= timeoutS-((10/100)*timeoutS)){
                    robot.motorFataDreapta.setPower(0.3);
                    robot.motorFataStanga.setPower(0.3);
                    robot.motorSpateDreapta.setPower(0.3);
                    robot.motorSpateStanga.setPower(0.3);
                }   else {
                    robot.motorFataDreapta.setPower(DRIVE_SPEED-0.2);
                    robot.motorFataStanga.setPower(DRIVE_SPEED-0.2);
                    robot.motorSpateDreapta.setPower(DRIVE_SPEED);
                    robot.motorSpateStanga.setPower(DRIVE_SPEED);
                }
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

            robot.motorFataDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorFataStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorSpateDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorSpateStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            robot.motorFataDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFataStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION

            sleep(200);   // optional pause after each move
        }
    }

    public void RearDrive(double FataDreapta,double FataStanga,double SpateDreapta,double SpateStanga, double timeoutS) {
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
            robot.motorFataDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorFataStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // reset the timeout time and start motion.

            robot.motorFataDreapta.setPower(-DRIVE_SPEED);
            robot.motorFataStanga.setPower(-DRIVE_SPEED);
            robot.motorSpateDreapta.setPower(-DRIVE_SPEED);
            robot.motorSpateStanga.setPower(-DRIVE_SPEED);
            runtime.reset();

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
            ) {
                if(runtime.seconds() >= timeoutS -((20/100)*timeoutS)){
                    robot.motorFataDreapta.setPower(-0.3);
                    robot.motorFataStanga.setPower(-0.3);
                    robot.motorSpateDreapta.setPower(-0.3);
                    robot.motorSpateStanga.setPower(-0.3);
                }
                else {
                    robot.motorFataDreapta.setPower(-DRIVE_SPEED);
                    robot.motorFataStanga.setPower(-DRIVE_SPEED);
                    robot.motorSpateDreapta.setPower(-DRIVE_SPEED);
                    robot.motorSpateStanga.setPower(-DRIVE_SPEED);
                }

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

            robot.motorFataDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorFataStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorSpateDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorSpateStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            robot.motorFataDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFataStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION

            sleep(200);   // optional pause after each move
        }
    }

    public void Prindere(double timeoutS) {
        // Ensure that the opmode is still active
        if (opModeIsActive()) {



            // Turn On RUN_TO_POSITION
            robot.motorFataDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorFataStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // reset the timeout time and start motion.

            robot.motorFataDreapta.setPower(DRIVE_SPEED);
            robot.motorFataStanga.setPower(DRIVE_SPEED);
            robot.motorSpateDreapta.setPower(DRIVE_SPEED);
            robot.motorSpateStanga.setPower(DRIVE_SPEED);
            runtime.reset();
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
            ) {

                    robot.motorFataDreapta.setPower(-0.3);
                    robot.motorFataStanga.setPower(-0.3);
                    robot.motorSpateDreapta.setPower(-0.3);
                    robot.motorSpateStanga.setPower(-0.3);

                // Display it for the driver.
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

            robot.motorFataDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorFataStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorSpateDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorSpateStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            robot.motorFataDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFataStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION

            sleep(200);   // optional pause after each move
        }
    }

    public void Apropiere(double timeoutS) {
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Turn On RUN_TO_POSITION
            robot.motorFataDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorFataStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            runtime.reset();
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
            ) {

                    robot.motorFataDreapta.setPower(-0.3);
                    robot.motorFataStanga.setPower(0.3);
                    robot.motorSpateDreapta.setPower(0.3);
                    robot.motorSpateStanga.setPower(-0.3);

                // Display it for the driver.
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

            robot.motorFataDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorFataStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorSpateDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorSpateStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            robot.motorFataDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFataStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorSpateDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorSpateStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION

            sleep(200);   // optional pause after each move
        }
    }

}
