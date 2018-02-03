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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


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

@TeleOp(name="Test 1", group="Linear Opmode")
//@Disabled
public class TestAutonomous extends LinearOpMode {

    boolean lastResetState = false;
    boolean curResetState  = false;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();

        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // If the A and B buttons are pressed just now, reset Z heading.
            curResetState = (gamepad1.a && gamepad1.b);
            if (curResetState && !lastResetState) {
                modernRoboticsI2cGyro.resetZAxisIntegrator();
            }
            lastResetState = curResetState;

            int rawX = modernRoboticsI2cGyro.rawX();
            int rawY = modernRoboticsI2cGyro.rawY();
            int rawZ = modernRoboticsI2cGyro.rawZ();
            int heading = modernRoboticsI2cGyro.getHeading();
            int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();

            // Read dimensionalized data from the gyro. This gyro can report angular velocities
            // about all three axes. Additionally, it internally integrates the Z axis to
            // be able to report an absolute angular Z orientation.
            AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
            float zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            // Read administrative information from the gyro
            int zAxisOffset = modernRoboticsI2cGyro.getZAxisOffset();
            int zAxisScalingCoefficient = modernRoboticsI2cGyro.getZAxisScalingCoefficient();

            telemetry.addLine()
                    .addData("dx", formatRate(rates.xRotationRate))
                    .addData("dy", formatRate(rates.yRotationRate))
                    .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));
            telemetry.addData("angle", "%s deg", formatFloat(zAngle));
            telemetry.addData("heading", "%3d deg", heading);
            telemetry.addData("integrated Z", "%3d", integratedZ);
            telemetry.addLine()
                    .addData("rawX", formatRaw(rawX))
                    .addData("rawY", formatRaw(rawY))
                    .addData("rawZ", formatRaw(rawZ));
            telemetry.addLine().addData("z offset", zAxisOffset).addData("z coeff", zAxisScalingCoefficient);
            telemetry.update();

            // Setup a variable for each drive wheel to save power level for telemetry

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.


            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            if(gamepad1.a) {
                leftDrive.setPower(0.3);
                rightDrive.setPower(-0.3);
            }
            else
            {
                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);
            }

            // Send calculated power to wheels

//            leftDrive.setPower(1.0);
//            rightDrive.setPower(1.0);
            telemetry.update();
        }
    }

    private void moveSpeed(DcMotor motor, float rotations, float time) {
        double angVel = (rotations / time) * 1120; // ticks per second
    }

    String formatRaw(int rawValue) {
        return String.format("%d", rawValue);
    }

    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatFloat(float rate) {
        return String.format("%.3f", rate);
    }
}
