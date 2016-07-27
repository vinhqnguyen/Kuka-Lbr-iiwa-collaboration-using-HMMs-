package backgroundTask;


import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.concurrent.TimeUnit;

import com.kuka.generated.ioAccess.DigOutIOGroup;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.PhysicalObject;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.userInterface.ServoMotionUtilities;

/**
 * This Background program collects some data for latter evaluation:
 * Position of the end-effector respected to Base Frame
 * External Force/Torque respected to Flange Frame
 * Author: Vinh Nguyen (vinhnguyen.ac@gmail.com)
 * Version: June 16, 2016
 */


public class BackgroundTask_writingResults extends
		RoboticsAPICyclicBackgroundTask {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr;
	private PhysicalObject _toolAttachedToLBR;
	public static int count = 0;
	private DigOutIOGroup DO;
	
	private PrintWriter writerData;
	private JointPosition JointPos;
	private Frame Pose;
	private Vector force, torque;
	private ForceSensorData data;
	double Tx, Ty, Tz, fx, fy, fz;
	long period = 40; // Recording period (40 milliseconds)

	
	
	// Initialize KUKA controller 
	public void initialize() {
		
		//initialize some tools and devices
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getRobot(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_14_R820_1");
		
        double[] translationOfTool = { 0, 0, 0 };
        double mass = 0;
        double[] centerOfMassInMillimeter = { 0, 0, 0 };
        
        _toolAttachedToLBR = ServoMotionUtilities.createTool(lbr,
                "SimpleJointMotionSampleTool", translationOfTool, mass,
                centerOfMassInMillimeter);
        
        DO = new DigOutIOGroup(kuka_Sunrise_Cabinet_1);
        
		
        //initialize Cyclic run
		initializeCyclic(0, period, TimeUnit.MILLISECONDS, CycleBehavior.BestEffort);
		
		
		//initialize writers
		try {
			// link to the files DataRecord.txt should be change if needed
			writerData = new PrintWriter("C:\\KRC\\ApplicationServer\\Git\\CollaborativeManipulation\\DataRecord.txt", "UTF-8");
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnsupportedEncodingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}

	
	
	// Run cyclically
	public void runCyclic() {
		
		count = count + 1;
		
		// Limit the recording time 
		if (count < 15000){ 
			
			// Output signal to see recording. This is not important but it is easy to check
			if (count % 20 == 0)
				DO.setDigOut_2(!DO.getDigOut_2());
			
			// Get force and torque
			data = lbr.getExternalForceTorque(lbr.getFlange()); // Force/Torque represent in flange frame
																
			force = data.getForce();
			torque = data.getTorque();
			
			//Get the position
			Pose = lbr.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame()); // Position represented in Base frame
			JointPos = lbr.getCurrentJointPosition();
			double J7 = JointPos.get(6);
			
			// Write the data
			writerData.println(System.currentTimeMillis() + " " 
					+ Pose.getX() + " " + Pose.getY() + " " + Pose.getZ() + " "
					+ Pose.getAlphaRad() + " " + Pose.getBetaRad() + " " + Pose.getGammaRad() + " " 
					+ force.getX() + " " + force.getY() + " " + force.getZ() + " " 
					+ torque.getX() + " " + torque.getY() + " " + torque.getZ() + " " 
					+ J7 + " " + JointPos.get(0) + " " + Running2HMMs.prediction + " " + SwitchingbasedForceMag.prediction_SF);

		}

	}
	
}
