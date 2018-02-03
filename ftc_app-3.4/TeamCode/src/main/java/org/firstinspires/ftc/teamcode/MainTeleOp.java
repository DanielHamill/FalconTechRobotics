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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Main Teleop", group="Iterative Opmode")
//@Disabled
public class MainTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor topLeft;
//    private DcMotor topRight;
//    private DcMotor bottomLeft;
//    private DcMotor bottomRight;

//    private DcMotor top;
//    private DcMotor bottom;
//    private DcMotor left;
//    private DcMotor right;

    private Servo armLeft;
    private Servo armRight;

    private DcMotor extendL;
    private DcMotor extendR;

    private DcMotor mainLeft;
    private DcMotor mainRight;

    //data
    double angleL = 0.0;
    double dAngleL = 0.0;
    double time = 0.0;
    double dTime = 0.0;
    double speedL = 0.0;

    double angleR = 0.0;
    double dAngleR = 0.0;
    double speedR = 0.0;

    double speedLT =0;
    double speedRT=0;
    double aveL =0;
    double aveR=0;
    int ite=0;

    double t1;
    int c1;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

//        topLeft  = hardwareMap.get(DcMotor.class, "tl");
//        topRight = hardwareMap.get(DcMotor.class, "tr");
//        bottomLeft = hardwareMap.get(DcMotor.class, "bl");
//        bottomRight = hardwareMap.get(DcMotor.class, "br");

//        top  = hardwareMap.get(DcMotor.class, "top");
//        bottom = hardwareMap.get(DcMotor.class, "bottom");
//        left = hardwareMap.get(DcMotor.class, "left");
//        right = hardwareMap.get(DcMotor.class, "right");



        mainLeft = hardwareMap.get(DcMotor.class, "left");
        mainRight = hardwareMap.get(DcMotor.class, "right");
        mainLeft.setDirection(DcMotor.Direction.REVERSE);

        extendL = hardwareMap.get(DcMotor.class, "exL");
        extendR = hardwareMap.get(DcMotor.class, "exR");

        armLeft = hardwareMap.get(Servo.class, "armL");
        armLeft.setPosition(-1);
        armRight = hardwareMap.get(Servo.class, "armR");
        armRight.setPosition(1);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

//        bottomRight.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
         //Setup a variable for each drive wheel to save power level for telemetry
        double leftPower = Range.clip(gamepad1.left_stick_y, -1, 1);
        double rightPower = Range.clip(gamepad1.right_stick_y, -1, 1);

        mainLeft.setPower(leftPower);
        mainRight.setPower(rightPower);

        if(gamepad2.left_trigger != 0.0) {
            extendL.setPower(-gamepad2.left_trigger);
            extendR.setPower(gamepad2.left_trigger);
        }
        else if(gamepad2.right_trigger != 0.0) {
            extendL.setPower(gamepad2.right_trigger);
            extendR.setPower(-gamepad2.right_trigger);
        }
        else{
            extendL.setPower(0.0);
            extendR.setPower(0.0);
        }

        if(gamepad2.a) {
            armLeft.setPosition(0.0);
            armRight.setPosition(0.95);
        }

        if(gamepad2.b) {
            armLeft.setPosition(0.45);
            armRight.setPosition(0.5);
        }

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
//        double y = gamepad1.left_stick_y;
//        double x = gamepad1.right_stick_x;
//        double z = gamepad1.right_trigger - gamepad1.left_trigger;
//
//        if(gamepad1.left_stick_x != 0.0) {
//            top.setPower(-gamepad1.left_stick_x);
//            bottom.setPower(gamepad1.left_stick_x);
//        }
//        else {
//            top.setPower(0.0);
//            bottom.setPower(0.0);
//        }
//
//        if(gamepad1.right_stick_y != 0.0) {
//            left.setPower(gamepad1.right_stick_y);
//            right.setPower(-gamepad1.right_stick_y);
//        }
//        else {
//            left.setPower(0.0);
//            right.setPower(0.0);
//        }
//
//        if(gamepad1.left_trigger != 0) {
//            top.setPower(gamepad1.left_trigger);
//            bottom.setPower(gamepad1.left_trigger);
//            left.setPower(gamepad1.left_trigger);
//            right.setPower(gamepad1.left_trigger);
//            return;
//        }
//
//        if(gamepad1.right_trigger != 0) {
//            top.setPower(gamepad1.right_trigger);
//            bottom.setPower(gamepad1.right_trigger);
//            left.setPower(gamepad1.right_trigger);
//            right.setPower(gamepad1.right_trigger);
//            return;
//        }





//        y = Range.clip(y, -1.0, 1.0) ;
//        x = Range.clip(x, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
//        if(gamepad1.left_trigger != 0) {
//            topLeft.setPower(gamepad1.left_trigger);
//            topRight.setPower(gamepad1.left_trigger);
//            bottomLeft.setPower(gamepad1.left_trigger);
//            bottomRight.setPower(gamepad1.left_trigger);
//            return;
//        }
//
//        if(gamepad1.right_trigger != 0) {
//            topLeft.setPower(gamepad1.right_trigger);
//            topRight.setPower(gamepad1.right_trigger);
//            bottomLeft.setPower(gamepad1.right_trigger);
//            bottomRight.setPower(gamepad1.right_trigger);
//            return;
//        }



        //for diagonal wheels
//        topLeft.setPower(Range.clip(y+x-z, -1.0, 1.0));
//        topRight.setPower(Range.clip(y-x+z, -1.0, 1.0));
//        bottomLeft.setPower(Range.clip(y-x-z, -1.0, 1.0));
//        bottomRight.setPower(Range.clip(y+x+z, -1.0, 1.0));
//
//
//
//        if(gamepad2.left_trigger != 0) {
//            extend.setPower(Range.clip(gamepad2.left_trigger, 0, 1));
//        }
//
//        if(gamepad2.right_trigger != 0) {
//            extend.setPower(Range.clip(gamepad2.right_trigger, 0, 1));
//        }

        // Show the elapsed game time and wheel power.


        aveL += speedL;
        aveR += speedR;
        c1++;
        //TODO: implement speed average
        if(runtime.milliseconds()> t1 + 1000) {
            t1 = runtime.milliseconds();
            aveL = 0;
            aveR = 0;
            c1 = 0;
        }

        dTime = time - runtime.milliseconds() / 1000;
        time = runtime.milliseconds() / 1000;
        //left speed calculations
        dAngleL = angleL - mainLeft.getCurrentPosition();
        angleL = mainLeft.getCurrentPosition();
        speedL = dTime != 0.0 ? dAngleL / dTime : 0.0;
        telemetry.addData("left instantaneous", speedL + " ticks/s");
        telemetry.addData("left average", aveL/c1 + " ticks/s");
        //right speed calculations
        dAngleR = angleR - mainRight.getCurrentPosition();
        angleR = mainRight.getCurrentPosition();

        speedR = dTime != 0.0 ? dAngleR / dTime : 0.0;


        telemetry.addData("right instantaneous", speedR + " ticks/s");
        telemetry.addData("right average", aveR/c1 + " ticks/s");
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }


}
