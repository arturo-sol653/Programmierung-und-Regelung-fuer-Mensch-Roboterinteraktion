package exercises;

import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import robot.ExecutionController;
import robot.SunriseConnector;
import utility.SingleInstanceChecker;

import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.BasicMotions;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import utility.DataHandler;

public class Task_03_ElbowConfiguration extends RoboticsAPIApplication {
	
	public Task_03_ElbowConfiguration(RoboticsAPIContext context){
		super(context); 
	}

	public static void main(String[] args) {
		RoboticsAPIContext.useGracefulInitialization(true);

		// check if another robot application is already running
		new SingleInstanceChecker().start();
		
		// initialization
		Task_03_ElbowConfiguration app = new Task_03_ElbowConfiguration(RoboticsAPIContext.createFromResource(Task_03_ElbowConfiguration.class, "RoboticsAPI.config.xml")); 
		SunriseConnector.initialize(app);

		app.run();
		
		ExecutionController.waitForAllMotionsFinished();
	}

	LBR robot;
	Tool tool;

	@Override
	public void run() {
		robot = SunriseConnector.getRobot();
		tool = SunriseConnector.getTool();
		System.out.println("Starting " + this.getClass().getSimpleName());

		/** load previously teached frames and joint positions */
		JointPosition J0 = DataHandler.loadJointPos("J5");
		JointPosition J1 = DataHandler.loadJointPos("J6");
		
		/** movements to joint positions and frames */
		System.out.println("Moving to JointPosition first_position");
		tool.move(BasicMotions.ptp(J0));
		Frame frame = robot.getCurrentCartesianPosition(tool.getDefaultMotionFrame());
		//Frame newFrame = Transformation.of(frame.getX(), frame.getY(), frame.getZ(), frame.getAlphaRad(), frame.getBetaRad(), frame.getGammaRad());
		Frame newFrame = frame.copy();

		System.out.println("Moving to JointPosition second_position");
		tool.move(BasicMotions.ptp(J1));

		System.out.println("Moving to back to Frame first_position");
		tool.move(BasicMotions.ptp(newFrame));
		/** wait until movements are finished [if there are async movements] */
		ExecutionController.waitForAllMotionsFinished();
		System.exit(0);
	}
}
