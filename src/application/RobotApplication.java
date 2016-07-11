package application;


import backgroundTask.BackgroundTask_writingResults;
import backgroundTask.Running2HMMs;
import backgroundTask.SwitchingbasedForceMag;

import com.kuka.generated.ioAccess.DigOutIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;

/**
 * This program resets the count and count_SF variables,
 * so that the program can write data again
 * Author: Vinh Quang Nguyen
 * 
 */
public class RobotApplication extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_14_R820_1;
	private DigOutIOGroup DO;

	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_14_R820_1 = (LBR) getRobot(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_14_R820_1");
		
		 DO = new DigOutIOGroup(kuka_Sunrise_Cabinet_1);

	}

	public void run() {
		
		//Reset the values of RunningHMMs.count, BackgroundTask_writingResults.count, and SwitchingbasedForceMag.count_SF
		int a = Running2HMMs.count;
		//System.out.println(a);
		
		Running2HMMs.count = 2;
		BackgroundTask_writingResults.count = 0;
		SwitchingbasedForceMag.count_SF = 2;
		
		System.out.println("Ready to write data force/torque again ");
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		RobotApplication app = new RobotApplication();
		app.runApplication();
	}
}
