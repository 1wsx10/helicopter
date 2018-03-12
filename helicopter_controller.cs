public IMyProgrammableBlock heliBlock = null;
public IMyTimerBlock timerBlock = null;

public string nextArg = "";

public enum State {waitingForArg, wait1, runPB, wait2, enableTimer}
public State state = 0;

public bool ready = true;


public Program() {

	if(!Me.CustomName.ToLower().Contains("helicontroller")) {
		Me.CustomName = "HeliController " + Me.CustomName;
	}

	// find the heli PB and timer
	List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
	GridTerminalSystem.GetBlocksOfType<IMyTerminalBlock>(blocks);

	foreach(IMyTerminalBlock block in blocks) {
		if(heliBlock == null && block is IMyProgrammableBlock && block.CustomName.ToLower().Contains("heli") && block != Me) {
			heliBlock = (IMyProgrammableBlock)block;
		}

		if(timerBlock == null && block is IMyTimerBlock && block.CustomName.ToLower().Contains("heli")) {
			timerBlock = (IMyTimerBlock)block;
		}

		if(heliBlock != null && timerBlock != null) {
			break;
		}
	}

	if(heliBlock == null) {
		Echo("No Helicopter PB found");
		ready = false;
	}

	if(timerBlock == null) {
		Echo("No Helicopter Timer found");
		ready = false;
	}
}

public void Main(string argument) {
	if(!ready) return;

	switch(state) {
		case State.waitingForArg:

			if(argument != "") {
				nextArg = argument;
				Runtime.UpdateFrequency = UpdateFrequency.Once;
				timerBlock.Enabled = false;
				state++;
				Echo("turned off timer");
			}
		break;
		case State.runPB:

			heliBlock.TryRun(nextArg);
			Runtime.UpdateFrequency = UpdateFrequency.Once;
			state++;
			Echo("ran PB");
		break;
		case State.enableTimer:

			timerBlock.Enabled = true;
			timerBlock.Trigger();
			state = 0;
			Echo("turned on timer");
		break;
		case State.wait1:

			Runtime.UpdateFrequency = UpdateFrequency.Once;
			state++;
		break;
		case State.wait2:

			Runtime.UpdateFrequency = UpdateFrequency.Once;
			state++;
		break;
	}
}