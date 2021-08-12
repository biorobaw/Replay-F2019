package com.github.biorobaw.scs_models.replay_f2019.old.tasks;

import com.github.biorobaw.scs.experiment.Experiment;
import com.github.biorobaw.scs.simulation.scripts.Script;
import com.github.biorobaw.scs.utils.files.XML;
import com.github.biorobaw.scs_models.replay_f2019.old.model.Model;

public class ActivateReplay implements Script{

	String subject;
	
	public ActivateReplay(XML xml) {
		subject = xml.getAttribute("subject_id");
	}
	
	@Override
	public void newTrial() {
		Model m = (Model)Experiment.get().getSubject(subject);
		m.setDoReplay(true);
	}
}
