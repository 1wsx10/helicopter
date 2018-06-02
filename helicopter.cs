public const string assistsArg = "assists";
public const string hoverArg = "auto hover";
public const string ignitionArg = "engine";


// turn on PID controllers for each axis
// these control rotation speed and have nothing to do with absolute speed
public bool pitch_assist = true;
public bool roll_assist = true;
public bool yaw_assist = true;

// make the helicopter try to self-level and reduce velocity to 0
// this controls absolute speed
public bool autohover = false;

//names of different blocks
//don't worry about the rotors which directly control the helicopter blades, those are detected automatically.
public const string controllerN = "Cockpit Forward";//cockpit
public const string mShaftN = "MRotor";//main rotor
public const string tShaftN = "TRotor";//tail rotor
public const string timerN = "Heli Timer Block";//timer block which triggers this PB



//increase this if your heli sinks when trying to hover
public const float collectiveDefault = 0.03f;


// multiply by -1 to reverse, or 0.5 to half, etc...
public const float overall_sensitivity = 1f;
public const float mouse_sensitivity = 0.09f;

public const float collectiveSensitivity 	= overall_sensitivity * 0.3f;

public const float mousepitch_sensitivity 	= overall_sensitivity * 1f;
public const float mouseyaw_sensitivity 	= overall_sensitivity * 1f;
public const float pitch_sensitivity 		= overall_sensitivity * 1.3f;
public const float yaw_sensitivity 			= overall_sensitivity * 1.3f;
public const float roll_sensitivity 		= overall_sensitivity * 1.3f;

// sensitivity adjustment when flight assists are active
public const float pid_sensitivity = 1f;



// control module joystick / gamepad bindings
// tell the game to use your joystick
// 			options > controls > Joystick or Gamepad
// type "/cm showinputs" into chat
// press move the desired axis
// put that name as it is in the quotes for the control you want, caps matter

// if there is a positive and negative version:
// 		use the first and second quotes in the pair, otherwise leave one set of quotes blank
// if there are 2 values for one name, like a joystick:
// 		put 'X' and 'Y' at the end to specify which axis you want

// i have entered the names for my joystick with the controls i want
public string[] jsPitchAxis = new string[] {"g.lsanalogY", ""}; //the pitch and roll are combined into one on my joystick, so i have to specify X or Y (at the end there)
public string[] jsYawAxis = new string[] {"g.rotz+analog", "g.rotz-analog"}; //the yaw on my joystick has positive and negative instead of one number
public string[] jsRollAxis = new string[] {"g.lsanalogX", ""};
public string[] jsCollectiveAxis = new string[] {"g.slider1-analog", "g.slider1+analog"};

// The game maps controls wrongly for a heli, and the program can't distinguish them.
// use 'disableNormalCMControls' to disable the controls above, but still apply the controls below.
// 		make the joystick unresponsive with the normal controls disabled
// 		then you can enable the normal controls and it should behave nicely.
public const bool disableNormalCMControls = false;
public string[] forward_back = new string[] {"", "g.lsanalogY"}; // none of the split ones on my joystick affect the game controls
public string[] right_left = new string[] {"", "g.lsanalogX"};
public string[] up_down = new string[] {"", ""};

// -1 to reverse, or 0.5 to half, etc...
public const float jsPitchAxis_sensitivity = 0.3f;
public const float jsYawAxis_sensitivity = 0.3f;
public const float jsRollAxis_sensitivity = 0.3f;
public const float jsCollectiveAxis_sensitivity = 0.3f;


// set to -1 for the fastest speed in the game (changes with mods)
public const float maxRotorRPM = -1f;























//--------------------------don't touch anything below here--------------------------

// default PID values
public const float pDefault = 0.2f;
public const float iDefault = 0.05f;
public const float dDefault = 0.15f;

// I value is multiplied by this, so you can make it slowly decay
public float idecay = 1f;

// "imertial measurement unit"
public Kinematics ShipIMU;
// for watching RPM
public Kinematics mainRotorMonitor;

//event log
public string log = "";

//weather we are using PID controllers at all (almost certainly)
public const bool enablePID = true;

//pid controllers for high level controls
public PID hoverYaw;
public PID hoverRoll;
public PID hoverCollective;

//...yep
public Helicopter theHelicopter;

// main rotor shaft
public IMyMotorStator mShaft;

// tShaft and tailRotor are the same thing
public IMyMotorStator tShaft;
public Rotor tailRotor;

public IMyShipController controller;
public IMyTimerBlock timer;

//weather control module is active (gets worked out by program)
public bool controlModule = true;

//weather we want the engine on
public bool engineEnabled = false;
//weather we just started the engine
public bool slowStart = false;
public PID slowStartController = null;
//used to figure out derivative of RPM for slow start
public float lastRPM = 0;
public float RPM = 0;


//just a math constant
public const float RADsToRPM = 30 / (float)Math.PI;

public bool wasUnderControl = false;


//lets just not use this...
public Program() {
	Runtime.UpdateFrequency = UpdateFrequency.Update100;
}
//use this instead
//weather we just compiled
public int justCompiled = 0;



// remove unreachable code warning
#pragma warning disable 0162

int counter = 0;
public void Main(string argument, UpdateType runType) {
	if(!Me.CustomName.ToLower().Contains("heli")) {
		Me.CustomName = "Heli " + Me.CustomName;
	}

	//shit breaks on the very first run for some reason
	if(justCompiled == 0) {
		justCompiled++;
		return;
	}

	if(justCompiled == 1) {
		if(!setup()) {
			Echo("Error in Setup");
			return;
		}
		justCompiled++;
	}
	writeBool = false;

	if(timer == null) {
		Echo("ERROR: no timer block to trigger me");
		return;
	}

	Echo($"{counter++}");

	//check cockpit
	bool imUnderControl = controller.IsAlive() && (controller.IsUnderControl || autohover);
	if(!imUnderControl) {
		//go to sleep
		Echo("sleep mode");
		Runtime.UpdateFrequency = UpdateFrequency.Update100;
		timer.Enabled = false;
		wasUnderControl = false;
		return;
	} else if(!wasUnderControl) {
		timer.Enabled = true;
	}
	Runtime.UpdateFrequency = UpdateFrequency.None;
	wasUnderControl = true;


	Echo($"{Runtime.TimeSinceLastRun}");
	if(Runtime.TimeSinceLastRun.Milliseconds > 16) {
		log += $"Time was greater than 0.016: \n\t{Runtime.TimeSinceLastRun.Milliseconds}ms";
	}
	Echo(log);


	// arguments

	//toggle assists
	if(argument.ToLower() == assistsArg.ToLower()) {
		timer.Enabled = true;

		pitch_assist = !pitch_assist;
		roll_assist = !roll_assist;
		yaw_assist = !yaw_assist;
	}

	//toggle hover
	if(argument.ToLower() == hoverArg.ToLower()) {
		timer.Enabled = true;

		autohover = !autohover;
	}

	//toggle engine
	if(argument.ToLower() == ignitionArg.ToLower()) {
		timer.Enabled = true;

		engineEnabled = !engineEnabled;
		mShaft.Enabled = engineEnabled;
		slowStart = engineEnabled;
		slowStartController = new PID(1, 1, 1, this);
		slowStartController.ClampI = false;
	}




	// detect movement, used for flight assists (PIDs)
	ShipIMU.Update(Runtime.TimeSinceLastRun.TotalSeconds);

	if(mainRotorMonitor == null) {
		mainRotorMonitor = new Kinematics(mShaft, mShaft.Top);
	} else {
		mainRotorMonitor.Update(Runtime.TimeSinceLastRun.TotalSeconds);

		RPM = (int)(mainRotorMonitor.VelocityAngularCurrent * RADsToRPM).Y;
	}

	if(RPM < 30) {
		// reset the PIDs
		theHelicopter.resetPIDs();
	}

	float maxTorque = mShaft.GetMaximum<float>("Torque");
	float maxVelocity = mShaft.GetMaximum<float>("Velocity");

	double lastRunTime = Runtime.TimeSinceLastRun.TotalSeconds;

	float dRPMdT = (float)((RPM - lastRPM) / lastRunTime);
	lastRPM = RPM;


	if(slowStart && RPM < 0.8 * maxVelocity) {
		//mShaft.Torque = 0.3f * (maxTorque - startTorque) * RPM / maxVelocity + startTorque;
		float slowStartTorque = (float)slowStartController.update(5, dRPMdT);
		if(!slowStartTorque.IsValid()) {
			slowStartTorque = 0;
		}
		mShaft.Torque = Math.Max(maxTorque * Math.Min(slowStartTorque, 1), 5000);
	} else {
		slowStart = false;
		mShaft.Torque = maxTorque;
	}
	//mShaft.Torque = 200000;

	//write($"slow start: {slowStart}\nT:{mShaft.Torque.Round(0)}");
	write($"main rotor RPM: {RPM}\n");
	write($"pitch assist: {pitch_assist}");
	write($"yaw assist: {yaw_assist}");
	write($"roll assist: {roll_assist}");
	write($"autohover: {autohover}");



	// control tail rotor pos to match main rotor pos
	tailRotor.setFromVec((mShaft.Top.WorldMatrix.Forward + mShaft.Top.WorldMatrix.Right / 1.414f).TransformNormal(mShaft.WorldMatrix.Invert()).TransformNormal(tShaft.WorldMatrix));





	// controls
	Vector3D translation = Vector3D.Zero;
	Vector3D rotation = Vector3D.Zero;



	apply_controls(ref translation, ref rotation);

	write(
		$@"
		Roll:       {progressBar((rotation.Z + 1) / 2)}
		Yaw:        {progressBar((rotation.Y + 1) / 2)}
		Pitch:      {progressBar((rotation.X + 1) / 2)}
		Collective: {progressBar((translation.Y + 1) / 2)}"
		);


	// write($"T: {translation.Round(1)}\nR: {rotation.Round(1)}");


	if(autohover) {
		apply_autohover(ref translation, ref rotation);
	}



	// set the blade angles
	theHelicopter.go(translation, rotation);
}



public string apply_controls(ref Vector3D translation, ref Vector3D rotation) {

	Vector3D movement = controller.MoveIndicator;



	// joystick / gamepad (requires control module)
	if(controlModule) {
		do {

			// setup control module
			Dictionary<string, object> inputs = new Dictionary<string, object>();
			try {
				inputs = Me.GetValue<Dictionary<string, object>>("ControlModule.Inputs");
				Me.SetValue<string>("ControlModule.AddInput", "all");
				Me.SetValue<bool>("ControlModule.RunOnInput", false);
				// Me.SetValue<int>("ControlModule.InputState", 1);
				// Me.SetValue<float>("ControlModule.RepeatDelay", 0.016f);
			} catch(Exception e) {
				controlModule = false;
				continue;
			}


			string errors = "";

			if(!disableNormalCMControls) {

				if(errors != null) { errors += "Normal Controls:";}
				rotation.X += applyControl(inputs, jsPitchAxis[0], 1f * jsPitchAxis_sensitivity, ref errors);
				rotation.X += applyControl(inputs, jsPitchAxis[1], -1f * jsPitchAxis_sensitivity, ref errors);

				rotation.Y += applyControl(inputs, jsYawAxis[0], 1f * jsYawAxis_sensitivity, ref errors);
				rotation.Y += applyControl(inputs, jsYawAxis[1], -1f * jsYawAxis_sensitivity, ref errors);

				rotation.Z += applyControl(inputs, jsRollAxis[0], 1f * jsRollAxis_sensitivity, ref errors);
				rotation.Z += applyControl(inputs, jsRollAxis[1], -1f * jsRollAxis_sensitivity, ref errors);

				translation.Y += applyControl(inputs, jsCollectiveAxis[0], 1f * jsCollectiveAxis_sensitivity, ref errors);
				translation.Y += applyControl(inputs, jsCollectiveAxis[1], -1f * jsCollectiveAxis_sensitivity, ref errors);
			} else {
				Vector3D move = controller.MoveIndicator;
				write(move.Round(1).ToString());

				if(errors != null) { errors += "Normal Controls Disabled for debug";}
			}


			if(errors != null) { errors += "\n\nMovement Counter:";}

			movement.Z += applyControl(inputs, forward_back[0], 1, ref errors);
			movement.Z += applyControl(inputs, forward_back[1], -1, ref errors);
			movement.X += applyControl(inputs, right_left[0], 1, ref errors);
			movement.X += applyControl(inputs, right_left[1], -1, ref errors);
			movement.Y += applyControl(inputs, up_down[0], 1, ref errors);
			movement.Y += applyControl(inputs, up_down[1], -1, ref errors);

			Echo(errors);

		} while(false);
	}




	// mouse + kb

	// collective
	translation.Y += movement.Y * collectiveSensitivity + collectiveDefault;

	// pitch
	rotation.X += controller.RotationIndicator.X * mousepitch_sensitivity * mouse_sensitivity * -1;
	rotation.X += movement.Z * pitch_sensitivity;
	if(pitch_assist) {
		rotation.X *= pid_sensitivity;
	}
	// yaw
	rotation.Y += controller.RotationIndicator.Y * mouseyaw_sensitivity * mouse_sensitivity;
	rotation.Y += movement.X * yaw_sensitivity;
	if(yaw_assist) {
		rotation.Y *= pid_sensitivity;
	}
	// roll
	rotation.Z += controller.RollIndicator * roll_sensitivity;
	if(roll_assist) {
		rotation.Z *= pid_sensitivity;
	}


	// return $"R: {rotation.Round(1)}\nT: {translation.Round(1)}";
	return null;
}


// returns value of inputs[key], and handles the vector2 thing
public double applyControl(Dictionary<string, object> inputs, string key, float multiplier) {
	string errors = null;
	return applyControl(inputs, key, multiplier, ref errors);
}


// writes to error string
// returns value of inputs[key], and handles the vector2 thing
public double applyControl(Dictionary<string, object> inputs, string key, float multiplier, ref string errors) {
	if(key == "") {
		if(errors != null) errors += $"\nKey Empty";
		return 0;
	}


	if(inputs.ContainsKey(key)) {
		if(inputs[key] is Vector2) {
			if(errors != null) errors += $"\nKey '{key}' is a vector, must specify X or Y";
		}
		// apply the value
		if(errors != null) errors += $"\nKey '{key}' {(float)inputs[key] * multiplier}";
		return (float)inputs[key] * multiplier;
	} else {
		string newkey = key.Substring(0, key.Length - 1);
		if(inputs.ContainsKey(newkey)) {
			if(!(inputs[newkey] is Vector2)) {
				// warn that its not a vector2 but they gave an X or Y value
				if(errors != null) errors += $"\nKey '{key}' not found";
				return 0;
			}
			if(is_X(key)) {
				// apply the X or Y
				if(errors != null) errors += $"\nKey '{key}' {((Vector2)inputs[newkey]).X * multiplier}";
				return ((Vector2)inputs[newkey]).X * multiplier;
			} else if(is_Y(key)) {
				// apply the X or Y
				if(errors != null) errors += $"\nKey '{key}' {((Vector2)inputs[newkey]).Y * multiplier}";
				return ((Vector2)inputs[newkey]).Y * multiplier;
			} else {
				// warn that there is no x or y, but the key minus the last char is valid
				if(errors != null) errors += $"\nKey '{key}' without last character matches but doesn't have X or Y";
				return 0;
			}
		} else {
			if(errors != null) errors += $"\nKey '{key}' {0}";
			return 0;
		}
	}
}


public bool is_X(string val) {
	if(val.ToLower()[val.Length - 1] == 'x') {
		return true;
	}
	return false;
}
public bool is_Y(string val) {
	if(val.ToLower()[val.Length - 1] == 'y') {
		return true;
	}
	return false;
}

public void apply_autohover(ref Vector3D translation, ref Vector3D rotation) {

		Vector3D counterRotation = Vector3D.Zero;
		Vector3D countertranslation = Vector3D.Zero;

		Vector3D grav = controller.GetNaturalGravity();

		grav = grav.TransformNormal(controller.WorldMatrix.Invert());

		// rotate around the X axis based on the Z axis of gravity and movement
		counterRotation.X += grav.Z * -0.1f;
		counterRotation.X += ShipIMU.VelocityLinearCurrent.Z * -1f / 50;

		// rotate around the Z axis based on the X axis of gravity
		counterRotation.Z += grav.X * -0.1f;



		// autohover PID controller for roll, this is separate to the assist PID controllers, measured is linear velocity rather than angular velocity
		if(hoverRoll == null) {
			hoverRoll = new PID(pDefault, iDefault - 0.02f, dDefault + 0.2f, this);
		} else {
			// rotate around the Z axis based on the X axis of movement
			counterRotation.Z += hoverRoll.update(0, ShipIMU.VelocityLinearCurrent.X * 1f / 50);
		}

		// autohover PID controller for collective, this is separate to the assist PID controllers, measured is linear velocity rather than angular velocity
		if(hoverCollective == null) {
			hoverCollective = new PID(pDefault, iDefault, dDefault, this);
		} else {

			// limit the hover collective based on the actual collective, this stops the PID from countering the users input
			hoverCollective.iLimit = 5 / ((translation.Y > 0 ? 1 : -1) + translation.Y * 5);

			// apply collective on the Y axis based on the movement along the Y axis
			countertranslation.Y += hoverCollective.update(0, ShipIMU.VelocityLinearCurrent.Y * 1f / 50);
		}


		// apply our calculations to the references we were given
		rotation += counterRotation;
		translation += countertranslation;
}


public bool setup() {
	bool isError = false;

	controller = (IMyShipController)GridTerminalSystem.GetBlockWithName(controllerN);
	if(controller == null) {
		Echo($"No controller with name '{controllerN}'");
		isError = true;
	}


	// get main rotors
	mShaft = (IMyMotorStator)GridTerminalSystem.GetBlockWithName(mShaftN);
	if(mShaft == null) {
		Echo($"No rotor found with name '{mShaftN}'");
		isError = true;
	}
	tShaft = (IMyMotorStator)GridTerminalSystem.GetBlockWithName(tShaftN);
	if(mShaft == null) {
		Echo($"No rotor found with name '{tShaftN}'");
		isError = true;
	}

	//get timer which keeps me alive
	timer = (IMyTimerBlock)GridTerminalSystem.GetBlockWithName(timerN);
	if(timer == null) {
		Echo($"No timer found with name '{timerN}'");
		isError = true;
	}

	
	if(isError) {
		return false;
	}

	//setup inertial measurement unit
	ShipIMU = new Kinematics((IMyEntity)controller, null);

	//allow us to control position of tail rotor
	tailRotor = new Rotor(tShaft);

	// construct swashplates
	SwashPlate mainSwash = new SwashPlate(this, controller, mShaft);
	SwashPlate antiTrq = new SwashPlate(this, controller, tShaft);


	//create swash plate objects for helicopter
	Dictionary<string, Pair<SwashPlate, IControlsConvert>> heliRotors = new Dictionary<string, Pair<SwashPlate, IControlsConvert>>();
	heliRotors.Add("Main Swashplate", new Pair<SwashPlate, IControlsConvert>(mainSwash, new mainRotorConvert()));
	heliRotors.Add("Anti Torque", new Pair<SwashPlate, IControlsConvert>(antiTrq, new antiTrqRotorConvert()));

	//init helicopter with swashplates
	theHelicopter = new Helicopter(this, controller, heliRotors);

	//set assists
	theHelicopter.is_PID_pitch_active = pitch_assist;
	theHelicopter.is_PID_yaw_active = yaw_assist;
	theHelicopter.is_PID_roll_active = roll_assist;
	theHelicopter.is_PID_tX_active = false;
	theHelicopter.is_PID_tY_active = false;
	theHelicopter.is_PID_tZ_active = false;

	return true;
}

public List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
private IMyTextPanel screen;
private bool writeBool;
public void write(string str) {
	str += "\n";
	if(writeBool) {
		screen.WritePublicText(str, true);
	} else {
		blocks.Clear();
		GridTerminalSystem.GetBlocksOfType<IMyTextPanel>(blocks);
		if(blocks.Count == 0) {
			return;
		}
		screen = (IMyTextPanel)blocks[0];
		for(int i = 0; i < blocks.Count; i++) {
			if(blocks[i].CustomName.IndexOf("%VectorLCD") != -1) {
				screen = (IMyTextPanel)blocks[i];
			}
		}
		screen.WritePublicText(str);
		writeBool = true;
	}
}

public static string progressBar(double val) {
	char[] bar = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
	for(int i = 0; i < 10; i++) {
		if(i <= val * 10) {
			bar[i] = '|';
		}
	}
	var str_build = new StringBuilder("[");
	for(int i = 0; i < 10; i++) {
		str_build.Append(bar[i]);
	}
	str_build.Append("]");
	return str_build.ToString();
}

// gets cos(angle between 2 vectors)
// cos returns a number between 0 and 1
// use Acos to get the angle
public static double angleBetweenCos(Vector3D a, Vector3D b) {
	double dot = Vector3D.Dot(a, b);
	double Length = a.Length() * b.Length();
	return dot/Length;
}

// gets cos(angle between 2 vectors)
// cos returns a number between 0 and 1
// use Acos to get the angle
// doesn't calculate length because thats expensive
public static double angleBetweenCos(Vector3D a, Vector3D b, double len) {
	double dot = Vector3D.Dot(a, b);
	return dot/len;
}


// set the angle to be between -pi and pi radians (0 and 360 degrees)
// this takes and returns radians
float cutAngle(float angle) {
	while(angle > Math.PI) {
		angle -= 2*(float)Math.PI;
	}
	while(angle < -Math.PI) {
		angle += 2*(float)Math.PI;
	}
	return angle;
}

// set the angle to be between -180 and 180 degrees
// this takes and returns radians
float cutAngleDegrees(float angle) {
	while(angle > 180) {
		angle -= 360;
	}
	while(angle < -180) {
		angle += 360;
	}
	return angle;
}

public class Pair<T, U> {
	public T first;
	public U second;

	public Pair(T first, U second) {
		this.first = first;
		this.second = second;
	}
}

public class Helicopter {

	public Program program;
	public IMyShipController controller;
	public Dictionary<string, Pair<SwashPlate, IControlsConvert>> heliRotors;

	public PID PID_pitch;
	public PID PID_yaw;
	public PID PID_roll;
	public PID PID_tX;
	public PID PID_tY;
	public PID PID_tZ;

	public bool is_PID_pitch_active = true;
	public bool is_PID_yaw_active = true;
	public bool is_PID_roll_active = true;
	public bool is_PID_tX_active = false;
	public bool is_PID_tY_active = false;
	public bool is_PID_tZ_active = false;

	public Helicopter(Program prog, IMyShipController cont, Dictionary<string, Pair<SwashPlate, IControlsConvert>> heliRotors) {
		this.program = prog;
		this.controller = cont;
		this.heliRotors = heliRotors;

		if(this.heliRotors == null) {
			this.heliRotors = new Dictionary<string, Pair<SwashPlate, IControlsConvert>>();
		}
	}

	public void resetPIDs() {

		if(PID_pitch != null) {
			PID_pitch.integral = 0;
		}
		if(PID_yaw != null) {
			PID_yaw.integral = 0;
		}
		if(PID_roll != null) {
			PID_roll.integral = 0;
		}
		if(PID_tX != null) {
			PID_tX.integral = 0;
		}
		if(PID_tY != null) {
			PID_tY.integral = 0;
		}
		if(PID_tZ != null) {
			PID_tZ.integral = 0;
		}
	}

	public void go(Vector3D translation, Vector3D rotation) {


		if(is_PID_pitch_active) {
			if(PID_pitch == null) {
				PID_pitch = new PID(pDefault, iDefault, dDefault, this.program);
			} else {
				rotation.X = PID_pitch.update(rotation.X, program.ShipIMU.VelocityAngularCurrent.X);
			}
		} else {
			PID_pitch = null;
		}
		if(is_PID_yaw_active) {
			if(PID_yaw == null) {
				PID_yaw = new PID(pDefault, iDefault, dDefault, this.program);
			} else {
				rotation.Y = PID_yaw.update(rotation.Y, program.ShipIMU.VelocityAngularCurrent.Y * -1);
			}
		} else {
			PID_yaw = null;
		}
		if(is_PID_roll_active) {
			if(PID_roll == null) {
				PID_roll = new PID(pDefault, iDefault, dDefault, this.program);
			} else {
				rotation.Z = PID_roll.update(rotation.Z, program.ShipIMU.VelocityAngularCurrent.Z * -1);
			}
		} else {
			PID_roll = null;
		}
		if(is_PID_tX_active) {
			if(PID_tX == null) {
				PID_tX = new PID(pDefault, iDefault, dDefault, this.program);
			} else {
				translation.X = PID_tX.update(translation.X, program.ShipIMU.VelocityLinearCurrent.X);
			}
		} else {
			PID_tX = null;
		}
		if(is_PID_tY_active) {
			if(PID_tY == null) {
				PID_tY = new PID(pDefault, iDefault, dDefault, this.program);
			} else {
				translation.Y = PID_tY.update(translation.Y, program.ShipIMU.VelocityLinearCurrent.Y);
			}
		} else {
			PID_tY = null;
		}
		if(is_PID_tZ_active) {
			if(PID_tZ == null) {
				PID_tZ = new PID(pDefault, iDefault, dDefault, this.program);
			} else {
				translation.Z = PID_tZ.update(translation.Z, program.ShipIMU.VelocityLinearCurrent.Z);
			}
		} else {
			PID_tZ = null;
		}

		foreach(var swash in heliRotors) {
			Controls cont = swash.Value.second.convert(translation, rotation);

			swash.Value.first.go(cont);
		}
	}
}


// this tells the program how to interpret the controls for each type of swashplate
public interface IControlsConvert {
	Controls convert(Vector3D translation, Vector3D rotation);
}

public class mainRotorConvert : IControlsConvert {
	public Controls convert(Vector3D translation, Vector3D rotation) {
		Controls output = new Controls();

		output.collective = translation.Y;
		output.cyclicF = rotation.X;
		output.cyclicR = rotation.Z * -1;

		return output;
	}
}

public class antiTrqRotorConvert : IControlsConvert {
	public Controls convert(Vector3D translation, Vector3D rotation) {
		Controls output = new Controls();

		// you turn the opposite direction of the tail rotor
		output.collective = rotation.Y * -1;
		output.cyclicF = 0;
		output.cyclicR = 0;

		return output;
	}
}

public struct Controls {
	public double collective;
	public double cyclicF;
	public double cyclicR;
}

public class SwashPlate {

	// physical parts
	public List<Rotor> blades;
	public IMyShipController controller;
	public IMyMotorStator rotorShaft;

	public Program prog;

	public string theStr = "";
	public bool printinfo = false;

	public float maxValue = 1f;


	public bool updateDirection = false;

	// auto-detect rotor blades
	public SwashPlate(Program prog, IMyShipController controller, IMyMotorStator rotorShaft) {

		// get all rotors whose grid is the same as the rotor shaft top grid
		List<IMyMotorStator> blocks = new List<IMyMotorStator>();
		prog.GridTerminalSystem.GetBlocksOfType<IMyMotorStator>(blocks, block => block.CubeGrid.EntityId == rotorShaft.TopGrid.EntityId);


		Setup(prog, controller, blocks.ToArray(), rotorShaft);
	}

	// only use the given rotor blades
	public SwashPlate(Program prog, IMyShipController controller, IMyMotorStator[] blades, IMyMotorStator rotorShaft) {
		Setup(prog, controller, blades, rotorShaft);
	}

	public void Setup(Program prog, IMyShipController controller, IMyMotorStator[] blades, IMyMotorStator rotorShaft) {
		this.controller = controller;
		this.rotorShaft = rotorShaft;
		this.prog = prog;

		updateDirection = Math.Abs(rotorShaft.TargetVelocityRPM) < 10;

		this.blades = new List<Rotor>();
		foreach(IMyMotorStator motor in blades) {
			Rotor current = new Rotor(motor);

			// forward direction is reversed if the main rotor is in reverse
			current.setForwardDir(Vector3D.Cross(rotorShaft.WorldMatrix.Up, motor.WorldMatrix.Up) * (rotorShaft.TargetVelocityRPM > 0 ? 1 : -1));
			// current.headOffset = current.localForwardDir; //no offset
			// current.headOffset = new Vector3D(-1,0,0);

			this.blades.Add(current);
		}
	}

	// controls, Vector3D

	// should keep controls between -1 and 1
	public void go(Controls cont) {

		if(updateDirection && Math.Abs(rotorShaft.TargetVelocityRPM) > 10) {
			foreach(Rotor blade in blades) {

				// forward direction is reversed if the main rotor is in reverse
				blade.setForwardDir(Vector3D.Cross(rotorShaft.WorldMatrix.Up, blade.theBlock.WorldMatrix.Up) * (rotorShaft.TargetVelocityRPM > 0 ? 1 : -1));

			}

			updateDirection = false;
		}

		if(printinfo) {
			theStr = "";
		}

		if(cont.collective > maxValue) {
			cont.collective = maxValue;
		} else if(cont.collective < -maxValue) {
			cont.collective = -maxValue;
		}
		if(cont.cyclicF > maxValue) {
			cont.cyclicF = maxValue;
		} else if(cont.cyclicF < -maxValue) {
			cont.cyclicF = -maxValue;
		}
		if(cont.cyclicR > maxValue) {
			cont.cyclicR = maxValue;
		} else if(cont.cyclicR < -maxValue) {
			cont.cyclicR = -maxValue;
		}

		if(printinfo) {
			theStr += "collective: " + cont.collective;
			theStr += "\ncyclicR: " + cont.cyclicR;
			theStr += "\ncyclicF: " + cont.cyclicF;
		}

		bool first = true;
		foreach(Rotor blade in blades) {

			Vector3D bladeForward = blade.getForwardDir();

			Vector3D vec =
				// point it forward
				bladeForward +
				// collective TODO find out why collective needs to be reversed
				rotorShaft.WorldMatrix.Up * cont.collective * -1 +
				// cyclic right
				Vector3D.Dot(bladeForward, rotorShaft.WorldMatrix.Right) * rotorShaft.WorldMatrix.Up * cont.cyclicR +
				// cyclic forward
				Vector3D.Dot(bladeForward, rotorShaft.WorldMatrix.Forward) * rotorShaft.WorldMatrix.Up * cont.cyclicF;

			blade.setFromVec(vec);

			if(printinfo && first) {
				first = false;

				theStr += "\nfirst basevec: " + blade.localForwardDir.ToString("0.0");
				theStr += "\nfirst desired: " + vec.ToString("0.0");
			}
		}
	}

	public Vector3D controlsToVec(Controls cont) {
		return new Vector3D(cont.cyclicF, cont.collective, cont.cyclicR * -1);
	}

	public Controls vecToControls(Vector3D vec) {
		Controls output = new Controls();
		output.collective = vec.Y;
		output.cyclicR = -1 * vec.Z;
		output.cyclicF = vec.X;
		return output;
	}
}

public class Rotor {

	public IMyMotorStator theBlock;

	public readonly float maxRotorRPM;

	public Vector3D localForwardDir = new Vector3D(0, 0, -1);
	// this should be the forward direction local to the rotor head
	public Vector3D headOffset = new Vector3D(0, 0, -1);

	public string theString;


	public Rotor(IMyMotorStator rotor) {
		this.theBlock = rotor;


		if(Program.maxRotorRPM <= 0) {
			maxRotorRPM	= rotor.GetMaximum<float>("Velocity");
		} else {
			maxRotorRPM = Program.maxRotorRPM;
		}
	}

	// sets the forward direction from world space
	public void setForwardDir(Vector3D worldForwardDir) {
		this.localForwardDir = worldForwardDir.TransformNormal(MatrixD.Invert(theBlock.WorldMatrix));
	}

	// gets the forward direction in world space
	public Vector3D getForwardDir() {
		return localForwardDir.TransformNormal(theBlock.WorldMatrix);
	}

	/*===| Part of Rotation By Equinox on the KSH discord channel. |===*/
	private void PointRotorAtVector(IMyMotorStator rotor, Vector3D targetDirection, Vector3D currentDirection, float multiplier) {
		double errorScale = Math.PI * maxRotorRPM;

		Vector3D angle = Vector3D.Cross(targetDirection, currentDirection);
		// Project onto rotor
		float err = (float)angle.Dot(rotor.WorldMatrix.Up);

		err *= (float)(errorScale * multiplier);
		// errStr += $"\nSETTING ROTOR TO {err:N2}";
		if (err > maxRotorRPM) {
			rotor.TargetVelocityRPM = maxRotorRPM;
		} else if ((err*-1) > maxRotorRPM) {
			rotor.TargetVelocityRPM = (maxRotorRPM * -1);
		} else {
			rotor.TargetVelocityRPM = err;
		}
	}

	// this sets the rotor to face the desired direction in worldspace
	// desiredVec doesn't have to be in-line with the rotors plane of rotation
	public double setFromVec(Vector3D desiredVec, float multiplier) {
		desiredVec = desiredVec.reject(theBlock.WorldMatrix.Up);
		desiredVec.Normalize();
		Vector3D currentDir = Vector3D.TransformNormal(this.headOffset, theBlock.Top.WorldMatrix);
		PointRotorAtVector(theBlock, desiredVec, currentDir/*theBlock.Top.WorldMatrix.Forward*/, multiplier);

		return angleBetweenCos(currentDir, desiredVec, desiredVec.Length());
	}

	public double setFromVec(Vector3D desiredVec) {
		return setFromVec(desiredVec, 1);
	}
}

public class PID {

	public Program prog;

	public float pmul = 1;
	public float imul = 0;
	public float dmul = 0;

	private double lasterror = 0;
	public double integral = 0;

	public double iLimit = 8;
	public bool ClampI = true;

	public string info = "";
	public bool printinfo = false;

	public PID(Program prog) {
		this.prog = prog;
	}

	public PID(float proportional, float integral, float derivative, Program prog) : this(prog) {
		this.pmul = proportional;
		this.imul = integral;
		this.dmul = derivative;
	}

	public double update(double setpoint, double measured) {
		double error = setpoint - measured;
		return update(error);
	}

	public double update(double error) {
		if(!enablePID) return error;

		double deltaT = prog.Runtime.TimeSinceLastRun.TotalMilliseconds;
		deltaT = (deltaT == 0 ? 1 : deltaT);

		integral *= prog.idecay;
		integral += error/deltaT;
		double derivative = (error - lasterror) / deltaT;
		lasterror = error;

		if(ClampI) {
			integral = Clamp(integral, iLimit, -1 * iLimit);
		}

		if(printinfo) {
			info =
				$@"P: {error * pmul}
				I: {integral * imul}
				D: {-1 * derivative * dmul}";
		}
		return error * pmul + integral * imul + -1 * derivative * dmul;
	}

	public static T Clamp<T>(T val, T a, T b) where T : IComparable<T> {
		T max;
		T min;

		int comp = a.CompareTo(b);
		if(comp > 0) {
			max = a;
			min = b;
		} else if(comp < 0) {
			max = b;
			min = a;
		} else {
			return a;
		}

		if(val.CompareTo(max) > 0) {
			val = max;
		}

		if(val.CompareTo(min) < 0) {
			val = min;
		}

		return val;
	}
}






/*

	Kinematics
	- release 1 (by plaYer2k)
	- Forum URL: http://forum.keenswh.com/threads/kinematics-kinematic-measurement-helper.7379961/

	This is a simple measurement class for linear and angular property transitions between continuous states.
	It offers the following properties(for more details, check the properties section within the Kinematics class):
		transition [m or rad] - the change from one state to another
		velocity [m/s or rad/s] - the first derivation of transition over time
		acceleration [m/s² or rad/s²] - the second derivation of transition over time
		jerk [m/s³ or rad/s³] - the third derivation of transition over time

	It can be used continously (one per game tick or second) as well as from one well defined state to another (irregular intervals).



	To initialize the Kinematics class you need the two parameters anchor and reference.

	The anchor is the entity whichs properties shall be checked. This has to be non-null.

	The reference is an optional entity to whichs orientation the anchor entity will be considered. This can be null.

	As example if you would use an anchor and no reference, you would track the anchors position in worldspace.
	If you however have anchor and reference present, you track the anchor relative to the reference.

	In the first case, you have the simpliest case. This is best used when you want to know your own ships properties.

	The second case however is useful for multi-grid applications. You could have a rotor as reference and its attached grid as anchor.
	In that case you could easily measure the rotations translation, speed, velocity and jerk.

	Alternative applications are:
		- tracking of astronauts
		- tracking of floating objects
		- tracking of friends/foes relative to you

*/
public class Kinematics
{

	public VRage.Game.ModAPI.Ingame.IMyEntity Anchor { get { return _anchor; } }
	public VRage.Game.ModAPI.Ingame.IMyEntity Reference { get { return _reference; } set { _reference = value; } }

	public MatrixD StateCurrent { get { return _stateCurrent; } }
	public MatrixD StateLast { get { return _stateLast; } }

	public MatrixD Transition { get { return _transition; } }

	public Vector3D TransitionLinearCurrent { get { return _transitionLinearCurrent; } }
	public Vector3D TransitionAngularCurrent { get { return _transitionAngularCurrent; } }
	public Vector3D TransitionLinearLast { get { return _transitionLinearLast; } }
	public Vector3D TransitionAngularLast { get { return _transitionAngularLast; } }

	public Vector3D VelocityLinearCurrent { get { return _velocityLinearCurrent; } }
	public Vector3D VelocityAngularCurrent { get { return _velocityAngularCurrent; } }
	public Vector3D VelocityLinearLast { get { return _velocityLinearLast; } }
	public Vector3D VelocityAngularLast { get { return _velocityAngularLast; } }

	public Vector3D AccelerationLinearCurrent { get { return _accelerationLinearCurrent; } }
	public Vector3D AccelerationAngularCurrent { get { return _accelerationAngularCurrent; } }
	public Vector3D AccelerationLinearLast { get { return _accelerationLinearLast; } }
	public Vector3D AccelerationAngularLast { get { return _accelerationAngularLast; } }

	public Vector3D JerkLinearCurrent { get { return _jerkLinearCurrent; } }
	public Vector3D JerkAngularCurrent { get { return _jerkAngularCurrent; } }
	public Vector3D JerkLinearLast { get { return _jerkLinearLast; } }
	public Vector3D JerkAngularLast { get { return _jerkAngularLast; } }

	// ~constants
	static readonly Vector3D VectorZero = new Vector3D(0, 0, 0);
	static readonly Vector3D VectorX = new Vector3D(1, 0, 0);
	static readonly Vector3D VectorY = new Vector3D(0, 1, 0);
	static readonly Vector3D VectorZ = new Vector3D(0, 0, 1);

	// Anchor and reference entities
	VRage.Game.ModAPI.Ingame.IMyEntity _anchor, _reference;

	// The stats current and last matrix. That is the orientation of the anchor relative to the reference
	MatrixD _stateCurrent, _stateLast;
	// The transition from the last state to the current state
	MatrixD _transition;
	// linear/angular transition values
	Vector3D _transitionLinearCurrent, _transitionAngularCurrent, _transitionLinearLast, _transitionAngularLast;
	// linear/angular velocity values
	Vector3D _velocityLinearCurrent, _velocityAngularCurrent, _velocityLinearLast, _velocityAngularLast;
	// linear/angular acceleration values
	Vector3D _accelerationLinearCurrent, _accelerationAngularCurrent, _accelerationLinearLast, _accelerationAngularLast;
	// linear/angular jerk values
	Vector3D _jerkLinearCurrent, _jerkAngularCurrent, _jerkLinearLast, _jerkAngularLast;

	public Kinematics(VRage.Game.ModAPI.Ingame.IMyEntity anchor, VRage.Game.ModAPI.Ingame.IMyEntity reference)
	{
		if(anchor == null)
			throw new Exception("Kinematics: anchor is null");

		this._anchor = anchor;
		this._reference = reference;

		if(reference == null)
			_transition = this._anchor.WorldMatrix;
		else
			_transition = this._anchor.WorldMatrix * MatrixD.Invert(this._reference.WorldMatrix);
		this._stateCurrent = this._stateLast = _transition;

		this._transitionLinearLast = this._transitionAngularLast = this._transitionLinearCurrent = this._transitionAngularCurrent = VectorZero;
		this._velocityLinearCurrent = this._velocityAngularCurrent = this._velocityLinearLast = this._velocityAngularLast = VectorZero;
		this._accelerationLinearCurrent = this._accelerationAngularCurrent = this._accelerationLinearLast = this._accelerationAngularLast = VectorZero;
		this._jerkLinearCurrent = this._jerkAngularCurrent = this._jerkLinearLast = this._jerkAngularLast = VectorZero;
	}

	public void Update(double deltaT)
	{
		this._transition = this._reference == null ? this._anchor.WorldMatrix : this._anchor.WorldMatrix * MatrixD.Invert(this._reference.WorldMatrix);
		this._stateLast = this._stateCurrent;
		this._stateCurrent = _transition;

		_transition = this._stateCurrent * MatrixD.Invert(this._stateLast);

		this._transitionLinearLast = this._transitionLinearCurrent;
		this._transitionLinearCurrent = _transition.Translation;

		this._transitionAngularLast = this._transitionAngularCurrent;
		this._transitionAngularCurrent = new Vector3D(
				System.Math.Asin(Vector3D.Dot(_transition.Up, VectorZ)),
				System.Math.Asin(Vector3D.Dot(_transition.Backward, VectorX)),
				System.Math.Asin(Vector3D.Dot(_transition.Right, VectorY))
			);

		this._velocityLinearLast = this._velocityLinearCurrent;
		this._velocityLinearCurrent = this._transitionLinearCurrent / deltaT;

		this._velocityAngularLast = this._velocityAngularCurrent;
		this._velocityAngularCurrent = this._transitionAngularCurrent / deltaT;

		this._accelerationLinearLast = this._accelerationLinearCurrent;
		this._accelerationLinearCurrent = (this._velocityLinearCurrent - this._velocityLinearLast) / deltaT;

		this._accelerationAngularLast = this._accelerationAngularCurrent;
		this._accelerationAngularCurrent = (this._velocityAngularCurrent - this._velocityAngularLast) / deltaT;

		this._jerkLinearLast = this._jerkLinearCurrent;
		this._jerkLinearCurrent = (this._accelerationLinearCurrent - this._accelerationLinearLast) / deltaT;

		this._jerkAngularLast = this._jerkAngularCurrent;
		this._jerkAngularCurrent = (this._accelerationAngularCurrent - this._accelerationAngularLast) / deltaT;
	}
}


}
public static class CustomProgramExtensions {

	public static bool IsAlive(this IMyTerminalBlock block) {
		if(block == null) {
			return false;
		}
		return block.CubeGrid.GetCubeBlock(block.Position)?.FatBlock == block;
	}

	// projects a onto b
	public static Vector3D project(this Vector3D a, Vector3D b) {
		double aDotB = Vector3D.Dot(a, b);
		double bDotB = Vector3D.Dot(b, b);
		return b * aDotB / bDotB;
	}

	public static Vector3D reject(this Vector3D a, Vector3D b) {
		return Vector3D.Reject(a, b);
	}

	public static Vector3D normalized(this Vector3D vec) {
		return Vector3D.Normalize(vec);
	}

	public static double dot(this Vector3D a, Vector3D b) {
		return Vector3D.Dot(a, b);
	}

	// get movement and turn it into worldspace
	public static Vector3D getWorldMoveIndicator(this IMyShipController cont) {
		return Vector3D.TransformNormal(cont.MoveIndicator, cont.WorldMatrix);
	}

	public static Vector3D TransformNormal(this Vector3D vec, MatrixD mat) {
		return Vector3D.TransformNormal(vec, mat);
	}

	public static MatrixD Invert(this MatrixD mat) {
		return MatrixD.Invert(mat);
	}

	public static string progressBar(this double val) {
		char[] bar = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
		for(int i = 0; i < 10; i++) {
			if(i <= val * 10) {
				bar[i] = '|';
			}
		}
		var str_build = new StringBuilder("[");
		for(int i = 0; i < 10; i++) {
			str_build.Append(bar[i]);
		}
		str_build.Append("]");
		return str_build.ToString();
	}

	public static string progressBar(this float val) {
		return ((double)val).progressBar();
	}

	public static string progressBar(this Vector3D val) {
		return val.Length().progressBar();
	}


	public static Vector3D Round(this Vector3D vec, int num) {
		return Vector3D.Round(vec, num);
	}

	public static double Round(this double val, int num) {
		return Math.Round(val, num);
	}

	public static float Round(this float val, int num) {
		return (float)Math.Round(val, num);
	}

	public static bool IsValid(this float val) {
		return (new Vector3D(val, val, val)).IsValid();
	}

	public static bool IsValid(this double val) {
		return (new Vector3D(val, val, val)).IsValid();
	}

