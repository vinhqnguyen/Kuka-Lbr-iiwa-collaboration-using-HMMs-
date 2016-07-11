package application;


import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import backgroundTask.Running2HMMs;
import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.fri.FRIConfiguration;
import com.kuka.connectivity.fri.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.PhysicalObject;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;

/**
 * Implementation of a robot application.
 * 
 * Description: This program arms to create different behavior of robot
 * with different impedance control parameter (rotation vs. translation)
 * 1. Running the robot with smart servor control
 * 2. Check if the require == rotation: the damper and moment inertia in rotation are low
 * 	  if the require == translation: the damper and mass in y direction are low
 *  
 *  Using HMMs for intention recognition
 *  Author: Vinh Quang Nguyen (vinhnguyen.ac@gmail.com)
 *  Version: June 16, 2016
 */
public class RotationVSTranslation_2HMMs extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr;
	private PhysicalObject _toolAttachedToLBR;
	private Tool gripper;
	public static Frame NextFrame = null;
    public static int _milliSleepToEmulateComputationalEffort = 10;
    private int _numRuns = 10000;
    
    //Impedance control parameters
    private static double mx = 0.5, my = 0.5; //mass in X and Y direction
    private static double bx = 2.0, by = 2.0; //damper in X and Y direction
    private static double cz = 0.5, iz = 0.08; //damper and moment inertia in Z direction
    
    //public static double bias = 0, relay = 0; // bias and relay in switching if necessary

    
    
    
    //Initializing function
	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getRobot(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_14_R820_1");
        // FIXME: Set proper Weights or use the plugin feature
        double[] translationOfTool =
        { 0, 0, 0 };
        double mass = 0;
        double[] centerOfMassInMillimeter =
        { 0, 0, 0 };
        _toolAttachedToLBR = ServoMotionUtilities.createTool(lbr,
                "SimpleJointMotionSampleTool", translationOfTool, mass,
                centerOfMassInMillimeter);
        
        
        gripper = getApplicationData().createFromTemplate("gripper");
        gripper.attachTo(lbr.getFlange());
        
	}

	
	
	// Run routine
	public void run(){
		
		
		//=============================================================================================//
		// 1. Recording data using FRI if it is needed.(this is an interface for online recording data)//
		//=============================================================================================//
    	String ip = "172.31.1.146";
    	FRIConfiguration config = FRIConfiguration.createRemoteConfiguration(lbr, ip);
    	config.setSendPeriodMilliSec(10);
    	//Initialize the FRI communication and automatically start it
    	FRISession session = new FRISession(config);
  

		
		
		
		//=============================================================================================//
		// 2. Moving to initial position                                                               //
    	//=============================================================================================//
		moveToInitialPosition();
		
		
		
		
		
		//=============================================================================================//
		// 3. Compliant reaction                                                                       //
		//=============================================================================================//
		
		// setup SmartServo interface 
		Frame initialPose = lbr.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame());
        Frame destFrame = initialPose;
        destFrame.setZ(initialPose.getZ());
        NextFrame = initialPose.copy();
        double z = NextFrame.getZ();
        
        SmartServo aSmartServoMotion = new SmartServo(lbr.getCurrentJointPosition());

        aSmartServoMotion.useTrace(true);

        aSmartServoMotion.setMinimumTrajectoryExecutionTime(5e-3);

        System.out.println("Starting RealtimeMotion in Position Mode");
        _toolAttachedToLBR.moveAsync(aSmartServoMotion);

        // Fetch the Runtime of the Motion part
        ISmartServoRuntime theServoRuntime = aSmartServoMotion.getRuntime();

        // Record the current Cartesian position
        Frame curPose = lbr.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame());
        
        
        // Run the SmartServo interface
		double deltaZ = 0;
		double x = 0, xd = 0, xdd = 0;
		double y = 0, yd = 0, ydd = 0;
		double alpha = 0, alpha_d = 0, alpha_dd = 0;
		double fX = 0, fY = 0, tZ = 0;
		double ForceX = 0, ForceY = 0, TorqueZ = 0;
		double dt = 0; //the sampling time
		double UL = 3, LL = -3; //dead zone for fx
		long startTimeStamp = System.currentTimeMillis(), endTimeStamp = System.currentTimeMillis();

		System.out.println("This robot is now in compliant control");
		
		for (double i = 0; i < _numRuns; ++i) {

			ThreadUtil.milliSleep(_milliSleepToEmulateComputationalEffort);

			// Get current position 
			curPose = lbr.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame());

			// Get interaction force/torque
			ForceSensorData data = lbr.getExternalForceTorque(lbr.getFlange()); // data is described in moving frame at end-effector
																				
			Vector force = data.getForce();
			Vector torque = data.getTorque();
			
			
			// Do calibration and implement dead-zone function for measured forces
			fX = (force.getX() - 3.6); // offset value of fx
			// dead zone function with LL = -3 and UL = 3
			if (fX < UL && fX > LL) ForceX = 0;
			if (fX > UL) ForceX = fX - UL;
			if (fX < LL) ForceX = fX - LL;

			fY = force.getY() + 3; // offset value of fy
			// dead zone function with LL = -3 and UL = 3
			if (fY < UL && fY > LL) ForceY = 0;
			if (fY > UL) ForceY = fY - UL;
			if (fY < LL) ForceY = fY - LL;

			tZ = torque.getZ();
			TorqueZ = tZ;
			
			// Switching mode base on the results of intention recognition in background task RunningHMMs
			if (Running2HMMs.prediction == 0) { // translation
				TorqueZ = 0;
			}

			if (Running2HMMs.prediction == 1) { // rotation
				ForceY = 0;
			}

			
			// Integrating differential equations to get reference position
			startTimeStamp = System.currentTimeMillis();
			dt = (double) ((startTimeStamp - endTimeStamp) / 1000.0);
			
			xdd = (ForceX - bx * xd) / mx;
			xd = xd + xdd * dt;
			x = 0 + xd * dt;

			ydd = (ForceY - by * yd) / my;
			yd = yd + ydd * dt;
			y = 0 + yd * dt;

			alpha_dd = (TorqueZ - cz * alpha_d) / iz;
			alpha_d = alpha_d + alpha_dd * dt;
			alpha = 0 + alpha_d * dt;

			
			// Set new destination
			deltaZ = z - curPose.getZ();
			destFrame.setParent(curPose);

			destFrame.setBetaRad(0);
			destFrame.setGammaRad(0);
			
			// Offset the error of orientation of end-effector 
			// (this is not necessary if the robot's end-effector's orientation keeps the same as desired)
			if (i % 5 == 0) {
				destFrame.setZ(-deltaZ);
			} 
			else destFrame.setZ(0);

			destFrame.setX(x * 1000); // set x in millimeter
			destFrame.setY(y * 1000); // set y in millimeter
			destFrame.setAlphaRad(alpha); // set alpha in rad

			theServoRuntime.setDestination(destFrame); //set destination

			endTimeStamp = System.currentTimeMillis(); //start measuring time for next step.
		
		} // end for loop
        

		//================================================================================================//
		// 4. Stop                                                                                        //
		//================================================================================================//
        // Stop the motion
        theServoRuntime.stopMotion();
        
        //Close the FRI communication
        session.close();
        
        System.out.println("Finished");
        
        
	}
	
	
	
	
	// move to initial position
	public void moveToInitialPosition()
	    {
	        lbr.move(
	                ptp(Math.toRadians(30), Math.PI / 180 * 0., 0., -Math.PI / 180 * 80., 0.,
	                        Math.PI / 180 * 100, Math.toRadians(30)).setJointVelocityRel(0.2));
	        /*
	         * Note: The Validation itself justifies, that in this very time
	         * instance, the load parameter setting was sufficient. This does not
	         * mean by far, that the parameter setting is valid in the sequel or
	         * lifetime of this program
	         */
	        ThreadUtil.milliSleep(1000);
	        if (!SmartServo.validateForImpedanceMode(lbr))
	        {
	            System.out
	                    .println("Validation of Torque Model failed - correct your mass property settings");
	            System.out
	                    .println("RealtimePTP will be available for position controlled mode only, until validation is performed");
	        }
	    }
 
	 
	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		RotationVSTranslation_2HMMs app = new RotationVSTranslation_2HMMs();
		app.runApplication();
	}
}
