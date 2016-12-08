package se.oru.aass.lucia2016.test;

import java.net.URI;

import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;


public class DebugMainClass {
	
	public static void main(String [ ] argv)
	  {

	    NodeMainExecutor executor = DefaultNodeMainExecutor.newDefault();

	    URI masterURI = null;

	    try {
	      masterURI = new URI("http://iran:11311/");
	    } catch(Exception ex) {
	      System.out.println(ex);
	    }

	    System.out.println("Starting node...");
	    NodeConfiguration iranConfig = NodeConfiguration.newPrivate();

	    iranConfig.setMasterUri(masterURI);

	    iranConfig.setNodeName("lucia_coordination");
	   NodeMain mainNode = new TestROSDispatching();
	    //NodeMain mainNode = new TestCancelMissionPublishers();
	    
	    executor.execute(mainNode, iranConfig);

	    System.out.println("Node started.");
	  }





}
