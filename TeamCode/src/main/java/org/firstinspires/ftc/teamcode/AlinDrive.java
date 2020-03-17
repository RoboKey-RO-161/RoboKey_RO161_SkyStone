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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
//Helo this is my sheet

@TeleOp(name="Basic: Driver Alin", group="Linear Opmode")

public class AlinDrive extends LinearOpMode {

    Hardware         robot   = new Hardware();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("status","Waiting for START" );
        telemetry.update();

        //declarations and mapping
        robot.init(hardwareMap);
        robot.motorBratStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBratDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBratStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBratDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double servospeed = 0.05;
        double poz1 =1.0;
        double poz2=0.0;

        int start1 = robot.motorBratStanga.getCurrentPosition();
        int start2 = robot.motorBratDreapta.getCurrentPosition();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.prindere.setPosition(0.8);
        robot.trapa1.setPosition(0.05);
        robot.trapa2.setPosition(0.95);

        robot.brat1.setPosition(poz1);
        robot.brat2.setPosition(poz2);

        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double motor1;
            double motor2;
            double motor3;
            double motor4;
//
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  = gamepad1.right_stick_x;
            double lateralStanga = gamepad1.left_stick_x;
            double lateraldreaptaconstruct = gamepad1.right_trigger;
            double lateralstangaconstruct = gamepad1.left_trigger;

            double brat11 = -gamepad2.right_stick_y;

            if(brat11 > 0.0) {
                robot.motorBratStanga.setTargetPosition(-9109);
                robot.motorBratDreapta.setTargetPosition(-9115);
            }
            if(brat11 < 0.0)
            {
                robot.motorBratDreapta.setTargetPosition(start2);
                robot.motorBratStanga.setTargetPosition(start1);
            }
            if(brat11 == 0.0)
            {
                robot.motorBratStanga.setTargetPosition(robot.motorBratStanga.getCurrentPosition());
                robot.motorBratDreapta.setTargetPosition(robot.motorBratDreapta.getCurrentPosition());
            }


            if(robot.motorBratStanga.getTargetPosition() == -9109)
            {
                robot.motorBratStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorBratStanga.setPower(0.8);
            }
            if(robot.motorBratStanga.getTargetPosition() == 0) {
                robot.motorBratStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorBratStanga.setPower(0.8);
            }
            if((robot.motorBratStanga.getTargetPosition() != -9109)&&(robot.motorBratStanga.getTargetPosition() != 0 )) {
                robot.motorBratStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorBratStanga.setPower(0);
            }
            if(robot.motorBratDreapta.getTargetPosition() == -9115)
            {
                robot.motorBratDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorBratDreapta.setPower(0.8);
            }
            if(robot.motorBratDreapta.getTargetPosition() == 0)
            {
                robot.motorBratDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorBratDreapta.setPower(0.8);
            }
            if((robot.motorBratDreapta.getTargetPosition() != -9115)&&(robot.motorBratDreapta.getTargetPosition() != 0) ) {
                robot.motorBratDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorBratDreapta.setPower(0);
            }
            if(gamepad2.y)
                robot.prindere.setPosition(1.0);
            else if(gamepad2.b)
                robot.prindere.setPosition(0.58);
            if(gamepad2.a){
                    robot.motor1intk.setPower(0.9);
                    robot.motor2intk.setPower(0.9);
                    }
            else if(gamepad2.x) {
                robot.motor1intk.setPower(-0.9);
                robot.motor2intk.setPower(-0.9);
            }
            else {
                    robot.motor1intk.setPower(0);
                    robot.motor2intk.setPower(0);
             }
            if(gamepad1.right_bumper ) {
                    robot.movebase1.setPosition(0.0);
                    robot.movebase2.setPosition(1.0);
            }
            else if(gamepad1.left_bumper) {
                    robot.movebase1.setPosition(1.0);
                    robot.movebase2.setPosition(0.0);
            }
            if(gamepad2.dpad_up)
            {
                robot.trapa1.setPosition(0.05);
                robot.trapa2.setPosition(0.95);
            }
            else if (gamepad2.dpad_left)
            {
                robot.trapa1.setPosition(0.6);
                robot.trapa2.setPosition(0.4);
            }
            else if(gamepad2.dpad_down)
            {
                robot.trapa1.setPosition(0.77);
                robot.trapa2.setPosition(0.23);
            }

           if(gamepad2.left_bumper) {
               poz1 += servospeed;
               poz2 -= servospeed;
               poz1 = Range.clip(poz1 , 0.0, 1.0);
               poz2 = Range.clip(poz2, 0.04,1.0);
           }
           else if(gamepad2.right_bumper){
               poz1 -= servospeed;
               poz2 += servospeed;
               poz1 = Range.clip(poz1 , 0.0, 1.0);
               poz2 = Range.clip(poz2 , 0.0,0.9);
           }
           if(gamepad2.dpad_right)
           {
               robot.brat1.setPosition(0.4);
               robot.brat2.setPosition(0.345);
           }
           else {
               robot.brat1.setPosition(poz1);
               robot.brat2.setPosition(poz2);
           }

            lateraldreaptaconstruct = Range.clip(lateraldreaptaconstruct, -0.3,0.3);
            lateralstangaconstruct = Range.clip(lateralstangaconstruct,-0.3,0.3);
            turn = Range.clip(turn, -0.4,0.4);


            motor1    = Range.clip((drive+turn)+(-lateralStanga+lateraldreaptaconstruct-lateralstangaconstruct), -0.9, 0.9) ;
            motor2    = Range.clip((drive+turn)+(+lateralStanga-lateraldreaptaconstruct+lateralstangaconstruct), -0.9, 0.9) ;
            motor3    = Range.clip((drive-turn)+(+lateralStanga-lateraldreaptaconstruct+lateralstangaconstruct), -0.9, 0.9);
            motor4    = Range.clip( (drive-turn)+(-lateralStanga+lateraldreaptaconstruct-lateralstangaconstruct), -0.9, 0.9);
            // Send calculated power to wheel

            robot.motorFataStanga.setPower(motor1);
            robot.motorSpateStanga.setPower(motor2);
            robot.motorFataDreapta.setPower(motor3);
            robot.motorSpateDreapta.setPower(motor4);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "fata stanga (%.2f), spate stanga(%.2f), fata dreapta(%.2f), spate dreapta(%.2f)", motor1, motor2, motor3, motor4);
            telemetry.addData("Status:", "motor brat stanga %7d motor brat dreapta %7d", robot.motorBratStanga.getCurrentPosition(),robot.motorBratDreapta.getCurrentPosition());
            telemetry.addData("Brat", "brat11 (%.2f)", brat11);
            telemetry.addData("Target", "stanga %7d dreapta %7d", robot.motorBratStanga.getTargetPosition(), robot.motorBratDreapta.getTargetPosition());
            telemetry.update();
        }
    }
}
