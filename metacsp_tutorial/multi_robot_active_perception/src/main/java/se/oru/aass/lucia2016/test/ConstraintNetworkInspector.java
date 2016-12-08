package se.oru.aass.lucia2016.test;

import java.io.File;

import javax.swing.JFileChooser;

import org.metacsp.framework.ConstraintNetwork;

public class ConstraintNetworkInspector {

	public static void main(String[] args) {

		JFileChooser chooser = new JFileChooser(System.getProperty("user.home") + ".ros");
		chooser.setMultiSelectionEnabled(true);
		chooser.setFileHidingEnabled(false);
		chooser.showOpenDialog(null);
		File[] files = chooser.getSelectedFiles();
		if (files != null && files.length > 0) {
			for (File file : files) {
				if (file.getName().endsWith(".cn")) {
					ConstraintNetwork con = ConstraintNetwork.loadConstraintNetwork(file);
					ConstraintNetwork.draw(con);
				}
			}
		}

	}

}
