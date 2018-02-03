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

import android.sax.TextElementListener;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.LinkedList;
import java.util.Queue;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="MainTestingAutonomous", group ="Concept")
//@Disabled
public class MainAutonomous extends LinearOpMode {

    //sensors
    VuMarkFinder vuMarkFinder;
    ColorSensor cs;
    Servo servo;
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    //data
    int step;
    private ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    double averageVel = 0;
    int aveCount = 0;

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

    LinkedList<Integer> positionsL;
    LinkedList<Integer> positionsR;
    double t1;
    double t2;


    //motors
    private DcMotor mainLeft;
    private DcMotor mainRight;

    @Override public void runOpMode() {
        //misc
        step = 0;
        mainLeft = hardwareMap.get(DcMotor.class, "left");
        mainRight = hardwareMap.get(DcMotor.class, "right");
        mainRight.setDirection(DcMotor.Direction.REVERSE);

        //servo
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(0.7);

        //sensor stuff
        cs = hardwareMap.get(ColorSensor.class, "csensor");
        cs.enableLed(true);
//        modernRoboticsI2cGyro.setHeadingMode(Heading);
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        // speed average
        positionsL = new LinkedList<Double>();
        positionsR = new LinkedList<Double>();
        t1 = 0;
        t2 = 0;

        //gyro calibration
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        //vu mark stuff
        String key = "AUZCki3/////AAAAGWWSNG4ohk4Aow4VrjhQb1AgxAftnASMYNiX4jJjV00q5+fxSuGgQ095QctEBAVMeCGqYi07l+9rUCOuXyUlmGvS8kXy1tHqnTU7Kn09HPHX3Iv9wMhhzUCdibORR9uRSJ8EyOQhEwtk48apuSZ7SwwU3G90T9+Baf+B6vJSadvtaEhZVqFYtghCch0+jdaREQkymHfFl7wobrLTHRzOZTHoh9RQWb/LgMf2ZP8UZVUHOfQE7gLHPC6waHIyZKCO5S0qvqL4a486bk/DqZPXZLWD/krL8QudHnX7JXPb5l+rhgA4poxh7Wx3fWmpYTQNpdtseWRGZLzPkwvQy+NNoO5e/JHQwRXSvD0AiWIufRKJ";
        vuMarkFinder = new VuMarkFinder(hardwareMap, key, telemetry);

        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.update();
        waitForStart();

        vuMarkFinder.start();


        waitForStart();
        double timerTenths = runtime.milliseconds();
        double timerHund = runtime.milliseconds();

        runtime.reset();
        telemetry.clear(); telemetry.update();

        double rotation = 0;

        int count;
        while (opModeIsActive()) {


            double angularVelocity = 0;
            float rawVel = gyro.getAngularVelocity(AngleUnit.DEGREES).zRotationRate;

            //turning test
//            if (runtime.milliseconds() > timerTenths + 100) {
//                timerTenths = runtime.milliseconds();
//                averageVel = 0;
//            }
//            if (runtime.milliseconds() > timerHund + 10) {
//                timerHund = runtime.milliseconds();
//                aveCount++;
//                averageVel += rawVel / aveCount;
//                rotation += averageVel / 100;
//            }


//            if(Math.abs(targetAngle) < currentAngle + targetAngle/** .93*/) {
//                mainLeft.setPower(0.3);
//                mainRight.setPower(-0.3);
//            }
//            else
//            {
//                mainLeft.setPower(0.0);
//                mainRight.setPower(0.0);
//            }

//            if(vuMarkFinder.getVuMark() == RelicRecoveryVuMark.LEFT) {
//                telemetry.addData("VuMark: Left", null);
//            }
//
//            if(vuMarkFinder.getVuMark() == RelicRecoveryVuMark.RIGHT) {
//                telemetry.addData("VuMark: Right", null);
//            }
//
//            if(vuMarkFinder.getVuMark() == RelicRecoveryVuMark.CENTER) {
//                telemetry.addData("VuMark: Center", null);
//            }
//
//            vuMarkFinder.getVuMark();
//
//            if(step == 0) {
//                timer = getRuntime();
//                servo.setPosition(0.0);
//                step++;
//            }
//
//            if(step == 1) {
//                if(getRuntime() > timer + 2) {
//                    timer = getRuntime();
//                    step++;
//                }
//            }
//
//            if(step == 2) {
//                if(cs.blue() >= 1) {
//                    mainLeft.setPower(0.3);
//                    mainRight.setPower(0.3);
//                }
//                else {
//                    mainLeft.setPower(-0.3);
//                    mainRight.setPower(-0.3);
//                }
//                step++;
//            }
//
//            if(step == 3) {
//                if(getRuntime() > timer + 0.7) {
//                    mainLeft.setPower(0.0);
//                    mainRight.setPower(0.0);
//                    step++;
//                }
//            }
//            if(step == 4) {
//                servo.setPosition(0.7);
//            }

            mainLeft.setPower(gamepad1.left_stick_y);
            mainRight.setPower(gamepad1.left_stick_y);

            //speed averages
            if(runtime.milliseconds() > t1 + 100) {
//                setSpeed(mainLeft, 2000, );
                t1 = runtime.milliseconds();
                positionsL.addFirst(mainLeft.getCurrentPosition());
                speedsR.addFirst(speedR);
                aveL += speedL;
                aveR += speedR;

                if(speedsL.size() > 20) {
                    aveL -= speedsL.removeLast();
                    aveR -= speedsR.removeLast();
                }
            }

            dTime = time - runtime.milliseconds() / 1000;
            time = runtime.milliseconds() / 1000;
            //left speed calculations
            dAngleL = angleL - mainLeft.getCurrentPosition();
            angleL = mainLeft.getCurrentPosition();
            speedL = dTime != 0.0 ? dAngleL / dTime : 0.0;
            telemetry.addData("left instantaneous", speedL + " ticks/s");
            telemetry.addData("left average", aveL/speedsL.size() + " ticks/s");
            telemetry.addData("size", speedsL.size());
            //right speed calculations
            dAngleR = angleR - mainRight.getCurrentPosition();
            angleR = mainRight.getCurrentPosition();

            speedR = dTime != 0.0 ? dAngleR / dTime : 0.0;


            telemetry.addData("right instantaneous", speedR + " ticks/s");
            telemetry.addData("right average", aveR/speedsR.size() + " ticks/s");
            telemetry.addData("size", speedsR.size());

//            double adjustedVel = averageVel > 0.01 ? averageVel : 0;
//            telemetry.addData("average angular velocity", adjustedVel);
//            telemetry.addData("angle", rotation);


            boolean bool = false;
            for(double num: speedsL) {
                if(num != 0) {
                    bool = true;
                }
            }
            if(bool) {
                telemetry.addData("0", "false");
            }
            else {
                telemetry.addData("0", "true");
            }



            telemetry.update();
        }
    }

    public void setSpeed(DcMotor motor, double speed, double goal, double initialPower) {

        if(motor.getPower() == 0.0) {
            motor.setPower(initialPower);
        }

        if(speed < goal) {
            motor.setPower(motor.getPower() + motor.getPower() * 0.005);
        }
        else if(speed > goal) {
            motor.setPower(motor.getPower() - motor.getPower() * 0.005);
        }

    }

}
