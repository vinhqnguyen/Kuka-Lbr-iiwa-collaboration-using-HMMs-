package com.kuka.roboticsAPI.directServo.samples;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.PhysicalObject;
import com.kuka.roboticsAPI.motionModel.DirectServo;
import com.kuka.roboticsAPI.motionModel.IDirectServoRuntime;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;

/**
 * Very simple sample of JointSpecific DirectServo Motion.
 * 
 * What you should learn in this lesson:
 * 
 * <ul>
 * <li>Activation of a DirectServo motion various control modes
 * <li>Modify compliance parameters during DirectServo motion
 * </ul>
 * You should already be familiar with
 * <ul>
 * <li>Sending a sequence joint specific setpoints
 * <li>Sending a sequence Cartesian specific setpoints
 * </ul>
 * NOTE: the control mode is independent of the goal setting part, so just joint
 * specific motion is discussed within this program - Left as student homework:
 * Perform the same to specify cartesian goals
 * 
 * @author schreiberg
 * 
 */
public class DirectServoSampleInteractionControl extends RoboticsAPIApplication
{
    // members
    private LBR _theLbr;
    /**
     * Will be initialized from the routine "ServoMotionUtilities.createTool()".
     * 
     * IMPORTANT NOTE: Set the load mass properly
     */
    private PhysicalObject _toolAttachedToLBR;

    @Override
    public void initialize()
    {
        // Locate the "first" Lightweight Robot in the system

        _theLbr = (LBR) ServoMotionUtilities.locateLBR(getContext());
        // FIXME: Set proper Weights or use the plugin feature
        final double[] translationOfTool =
        { 0, 0, 100 };
        final double mass = 0;
        final double[] centerOfMassInMillimeter =
        { 0, 0, 100 };
        _toolAttachedToLBR = ServoMotionUtilities.createTool(_theLbr,
                "SimpleJointMotionSampleTool", translationOfTool, mass,
                centerOfMassInMillimeter);

    }

    /**
     * Move to an initial Position WARNING: MAKE SURE, THAT the pose is
     * collision free.
     */
    public void moveToInitialPosition()
    {
        _toolAttachedToLBR.move(
                ptp(0., Math.PI / 180 * 30., 0., -Math.PI / 180 * 60., 0.,
                        Math.PI / 180 * 90., 0.).setJointVelocityRel(0.1));
        /*
         * Note: The Validation itself justifies, that in this very time
         * instance, the load parameter setting was sufficient. This does not
         * mean by far, that the parameter setting is valid in the sequel or
         * lifetime of this program
         */
        ThreadUtil.milliSleep(1000);
        if (DirectServo.validateForImpedanceMode(_toolAttachedToLBR) == false)
        {
            System.out
                    .println("Validation of Torque Model failed - correct your mass property settings");
            System.out
                    .println("DirectServo will be available for position controlled mode only, until validation is performed");
        }
    }

    // Sleep in between
    private int _milliSleepToEmulateComputationalEffort = 0;
    private final int _numRuns = 500;
    private final double _amplitude = 0.2;
    private final double _freqency = 0.1;

    /**
     * 
     * @param controlMode
     */
    public void runDirectServoMotion(final IMotionControlMode controlMode)
    {

        final JointPosition initialPosition = new JointPosition(
                _theLbr.getCurrentJointPosition());

        final DirectServo aDirectServoMotion = new DirectServo(initialPosition);

        System.out.println("Starting RealtimeMotion in "
                + controlMode.getClass().getName());
        aDirectServoMotion.setMinimumTrajectoryExecutionTime((double) ((8 + _milliSleepToEmulateComputationalEffort)) / 1000.);
        // Set the control mode as member of the realtime motion

        //        SunriseExecutionService.printSPRs(true);
        _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(
                aDirectServoMotion.setMode(controlMode));

        // Fetch the Runtime of the Motion part
        // NOTE: the Runtime will exist AFTER motion command was issued
        final IDirectServoRuntime theServoRuntime = aDirectServoMotion
                .getRuntime();

        // create an JointPosition Instance, to play with
        final JointPosition destination = new JointPosition(
                _theLbr.getJointCount());

        // For Roundtrip time measurement...
        final StatisticTimer timing = new StatisticTimer();
        try
        {
            // do a cyclic loop
            // Refer to some timing...
            // in nanosec
            final double omega = _freqency * 2 * Math.PI * 1e-9; // freqency * ...
            final long startTimeStamp = System.nanoTime();

            for (int i = 0; i < _numRuns; ++i)
            {
                // Timing - draw one step
                final OneTimeStep aStep = timing.newTimeStep();
                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // emulate some computational effort - or waiting for external
                // stuff
                ThreadUtil.milliSleep(_milliSleepToEmulateComputationalEffort);
                //
                theServoRuntime.updateWithRealtimeSystem();
                // Get the measured position in cartesian...
                final JointPosition curMsrJntPose = theServoRuntime
                        .getAxisQMsrOnController();

                final double curTime = System.nanoTime() - startTimeStamp;
                final double sinArgument = omega * curTime;

                for (int k = 0; k < destination.getAxisCount(); ++k)
                {
                    destination.set(k, Math.sin(sinArgument)
                            * _amplitude + initialPosition.get(k));
                }
                theServoRuntime
                        .setDestination(destination);

                //
                // Modify the stiffness settings every now and then
                //
                if (i % (_numRuns / 10) == 0)
                {
                    // modify stiffness settings
                    // update realtime system
                    if (controlMode instanceof CartesianImpedanceControlMode)
                    {
                        // We are in CartImp Mode,
                        // Modify the settings:
                        // NOTE: YOU HAVE TO REMAIN POSITIVE SEMI-DEFINITE !!
                        // NOTE: DONT CHANGE TOO FAST THE SETTINGS, ELSE YOU
                        // WILL DESTABILIZE THE CONTROLLER
                        final CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) controlMode;
                        final double aTransStiffVal = Math.max(100. * (i
                                / (double) _numRuns + 1), 1000.);
                        final double aRotStiffVal = Math.max(10. * (i
                                / (double) _numRuns + 1), 150.);
                        cartImp.parametrize(CartDOF.TRANSL).setStiffness(aTransStiffVal);
                        cartImp.parametrize(CartDOF.ROT).setStiffness(aRotStiffVal);
                        // Send the new Stiffness settings down to the
                        // controller
                        theServoRuntime
                                .changeControlModeSettings(cartImp);
                    }

                }

                aStep.end();

            } // end for
        }
        catch (final Exception e)
        {
            System.out.println(e);
            e.printStackTrace();
        }

        // /////////////////////////////////////////////////
        // print statistics and parameters of the motion
        System.out.println("Displaying final states after loop "
                + controlMode.getClass().getName());

        System.out.println(getClass().getName() + "\n" + theServoRuntime.toString());
        // Stop the motion
        theServoRuntime.stopMotion();
        System.out.println("Statistic Timing of Overall Loop " + timing);
        if (timing.getMeanTimeMillis() > 150)
        {
            System.out
                    .println("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
            System.out
                    .println("You should check the TCP/IP Stack Configuration - see the manual for details");
        }

    }

    protected CartesianImpedanceControlMode createCartImp()
    {
        final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
        cartImp.parametrize(CartDOF.TRANSL).setStiffness(1000.0);
        cartImp.parametrize(CartDOF.ROT).setStiffness(100.0);
        cartImp.setNullSpaceStiffness(100.);
        // For your own safety, shrink the motion abilities to useful limits
        cartImp.setMaxPathDeviation(50., 50., 50., 50., 50., 50.);
        return cartImp;
    }

    /** Sample to switch the motion control mode */
    protected void switchMotionControlMode()
    {
        System.out.println("Switch Motion Control Mode Sample");
        final boolean debugPrintoutFlag = false;
        /* Prepare two control modes for the sample */
        final CartesianImpedanceControlMode cartImp = createCartImp();

        final JointPosition initialPosition = new JointPosition(
                _theLbr.getCurrentJointPosition());

        final DirectServo firstDirectServoMotion = new DirectServo(initialPosition);

        firstDirectServoMotion.setMode(cartImp);
        System.out.println("Starting RealtimeMotion in "
                + cartImp.getClass().getName());

        // Set the control mode as member of the realtime motion
        _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(firstDirectServoMotion);

        // Fetch the Runtime of the Motion part
        // NOTE: the Runtime will exist AFTER motion command was issued
        final IDirectServoRuntime theFirstRuntime = firstDirectServoMotion
                .getRuntime();

        /*
         * Do Interaction with that mode Run set points etc...
         */
        theFirstRuntime.setDestination(initialPosition);

        /* Here: Just wait, until fine interpolation has finished */
        while (theFirstRuntime.isDestinationReached() != true)
        {
            ThreadUtil.milliSleep(_milliSleepToEmulateComputationalEffort);
            theFirstRuntime.updateWithRealtimeSystem();
            if (debugPrintoutFlag)
            {
                System.out.println("Waiting for reaching goal "
                        + theFirstRuntime);
            }
        }

        // ///////////////////
        // Open second Motion
        // ////

        // initialize another goal position position
        //
        for (int i = 0; i < initialPosition.getAxisCount(); i++)
        {
            initialPosition.set(i, initialPosition.get(i) + 0.1);
        }

        final DirectServo secondDirectServoMotion = new DirectServo(initialPosition);

        // / Activate the Motion -- it will become truely active, as the first
        // one vanishes
        // Set the control mode as member of the realtime motion
        _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(secondDirectServoMotion);

        System.out.println("Now blending over to -> Sending Stop Request "
                );

        // ///////////////////////////////////
        // Now blend over - stop the first,
        // the second will immediately take over
        theFirstRuntime.stopMotion();
        // get the runtime of the second motion
        //
        final IDirectServoRuntime theSecondRuntime = secondDirectServoMotion
                .getRuntime();
        theSecondRuntime.setDestination(initialPosition);
        /*
         * do further computations... on theSecondRuntime
         */

        /* Here: Just wait, until fine interpolation has finished */
        while (theSecondRuntime.isDestinationReached() != true)
        {
            ThreadUtil.milliSleep(_milliSleepToEmulateComputationalEffort);
            theSecondRuntime.updateWithRealtimeSystem();
            if (debugPrintoutFlag)
            {
                System.out.println("Waiting for reaching goal Runtime 2"
                        + theSecondRuntime);
            }
        }

        theSecondRuntime.stopMotion();

        System.out.println("\nResult of Motion 1" + theFirstRuntime);
        System.out.println("Result of Motion 2" + theSecondRuntime);

        System.out.println("Bye for now");

    }

    @Override
    public void run()
    {
        // ///////////////////////////////////////////////////////////////////////////////////////////////
        // Cartesian impedance sample
        // ///////////////////////////////////////////////////////////////////////////////////////////////
        moveToInitialPosition();

        // Start with cartesian impedance control mode
        //
        // Initialize Cartesian impedance mode
        //
        final CartesianImpedanceControlMode cartImp = createCartImp();
        // run the direct servo motion, as before done in the SimpleJointMotion
        // sample.
        // the only difference - pass on the InteractionControlStrategy...
        runDirectServoMotion(cartImp);

        // ///////////////////////////////////////////////////////////////////////////////////////////////
        // For completeness: Position Control sample
        // ///////////////////////////////////////////////////////////////////////////////////////////////

        // Return to initial position
        moveToInitialPosition();
        final PositionControlMode positionCtrlMode = new PositionControlMode();
        runDirectServoMotion(positionCtrlMode);
        // ///////////////////////////////////////////////////////////////////////////////////////////////
        //
        // Sample to switch control modes
        //
        // ///////////////////////////////////////////////////////////////////////////////////////////////
        moveToInitialPosition();
        switchMotionControlMode();

    }

    /**
     * Main routine, which starts the application
     */
    public static void main(final String[] args)
    {
        final DirectServoSampleInteractionControl app = new DirectServoSampleInteractionControl();

        app.runApplication();

    }
}
