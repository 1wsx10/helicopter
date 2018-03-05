
public bool collective_assist = false;
public bool pitch_assist = true;
public bool roll_assist = true;
public bool yaw_assist = true;

public string log = "";

public const bool swapPitchAndRoll = false;

public const bool enablePID = true;
public float idecay = 1f;

// "imertial measurement unit"
Kinematics ShipIMU;

public float last_pitch_control = 0;

public bool idleUP = false;


IMyMotorStator mShaft;
// tShaft and tailRotor are the same thing
IMyMotorStator tShaft;
Rotor tailRotor;

IMyShipController controller;

public const float RPMtoRADs = (float)Math.PI / 30;



public const float mouseSpeed = 0.09f;

public const float collectiveDefault = 0.25f;
public const float collectiveSensitivity = 0.3f;

public const float mousepitch_sensitivity = 1f;
public const float mouseyaw_sensitivity = 1f;
public const float pitch_sensitivity = 1.3f;
public const float yaw_sensitivity = 1.3f;
public const float roll_sensitivity = 1.3f;



Helicopter theHelicopter;

public const float pDefault = 0.2f;
public const float iDefault = 0.05f;
public const float dDefault = 0.15f;











public Program() {
	setup();
}


// remove unreachable code warning
#pragma warning disable 0162

int counter = 0;
public void Main(string argument) {
	writeBool = false;

	Echo($"{counter++}");

	Echo($"{Runtime.TimeSinceLastRun}");
	if(Runtime.TimeSinceLastRun.Milliseconds > 16) {
		log += $"Time was greater than 0.016: \n\t{Runtime.TimeSinceLastRun.Milliseconds}ms";
	}
	Echo(log);





	// detect movement, used for flight assists (PIDs)
	ShipIMU.Update(Runtime.TimeSinceLastRun.TotalSeconds);

	// string output ="";
	// output += string.Format("Velocity (Linear) [m/s]\n{0}\nVelocity (Angular) [rad/s]\n{1}\n",
	// 		ShipIMU.VelocityLinearCurrent.ToString("0.000\n"),
	// 		ShipIMU.VelocityAngularCurrent.ToString("0.000\n")
	// 	);
	// output += string.Format("Acceleration (Linear) [m/s²]\n{0}\nAcceleration (Angular) [rad/s²]\n{1}",
	// 		ShipIMU.AccelerationLinearCurrent.ToString("0.00\n"),
	// 		ShipIMU.AccelerationAngularCurrent.ToString("0.00\n")
	// 	);
	// output += string.Format("Jerk (Linear) [m/s³]\n{0}\nJerk (Angular) [rad/s³]\n{1}\n",
	// 		ShipIMU.JerkLinearCurrent.ToString("0.000\n"),
	// 		ShipIMU.JerkAngularCurrent.ToString("0.000\n")
	// 	);
	// write(output);



	// control tail rotor speed to match main rotor speed
	tailRotor.setFromVec((mShaft.Top.WorldMatrix.Forward + mShaft.Top.WorldMatrix.Right / 1.414f).TransformNormal(mShaft.WorldMatrix.Invert()).TransformNormal(tShaft.WorldMatrix));





	// controls
	Vector3D translation = Vector3D.Zero;
	Vector3D rotation = Vector3D.Zero;


	// collective
	translation.Y = controller.MoveIndicator.Y * collectiveSensitivity + collectiveDefault;

	// pitch
	rotation.X = controller.RotationIndicator.X * mousepitch_sensitivity * mouseSpeed * -1 +
		controller.MoveIndicator.Z * pitch_sensitivity;
	// yaw
	rotation.Y = controller.RotationIndicator.Y * mouseyaw_sensitivity * mouseSpeed +
		controller.MoveIndicator.X * yaw_sensitivity;
	// roll
	rotation.Z = controller.RollIndicator * roll_sensitivity;





	// set the blade angles
	theHelicopter.go(translation, rotation);
}


public bool setup() {

	// TODO: make this check

	controller = (IMyShipController)GridTerminalSystem.GetBlockWithName("Cockpit Forward");

	ShipIMU = new Kinematics((IMyEntity)controller, null);

	// get rotors
	mShaft = (IMyMotorStator)GridTerminalSystem.GetBlockWithName("MRotor");
	tShaft = (IMyMotorStator)GridTerminalSystem.GetBlockWithName("TRotor");

	IMyMotorStator[] mainSwashRotors = new IMyMotorStator[] {
		(IMyMotorStator)GridTerminalSystem.GetBlockWithName("MRotor A"),
		(IMyMotorStator)GridTerminalSystem.GetBlockWithName("MRotor B"),
		(IMyMotorStator)GridTerminalSystem.GetBlockWithName("MRotor C"),
		(IMyMotorStator)GridTerminalSystem.GetBlockWithName("MRotor D")
	};

	IMyMotorStator[] antiTrqRotors = new IMyMotorStator[] {
		(IMyMotorStator)GridTerminalSystem.GetBlockWithName("TRotor A"),
		(IMyMotorStator)GridTerminalSystem.GetBlockWithName("TRotor B"),
		(IMyMotorStator)GridTerminalSystem.GetBlockWithName("TRotor C"),
		(IMyMotorStator)GridTerminalSystem.GetBlockWithName("TRotor D")
	};

	tailRotor = new Rotor(controller, tShaft);

	// construct swashplates
	SwashPlate mainSwash = new SwashPlate(controller, mainSwashRotors, mShaft);
	SwashPlate antiTrq = new SwashPlate(controller, antiTrqRotors, tShaft);


	Dictionary<string, Pair<SwashPlate, IControlsConvert>> heliRotors = new Dictionary<string, Pair<SwashPlate, IControlsConvert>>();

	heliRotors.Add("Main Swashplate", new Pair<SwashPlate, IControlsConvert>(mainSwash, new mainRotorConvert()));
	heliRotors.Add("Anti Torque", new Pair<SwashPlate, IControlsConvert>(antiTrq, new antiTrqRotorConvert()));

	theHelicopter = new Helicopter(this, controller, heliRotors);

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


public class PID {

	public Program prog;

	public float pmul = 1;
	public float imul = 0;
	public float dmul = 0;

	private double lasterror = 0;
	private double integral = 0;

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

	public string theStr = "";
	public bool printinfo = false;

	public float maxValue = 1f;

	public SwashPlate(IMyShipController controller, IMyMotorStator[] blades, IMyMotorStator rotorShaft) {
		this.controller = controller;
		this.rotorShaft = rotorShaft;

		this.blades = new List<Rotor>();
		foreach(IMyMotorStator motor in blades) {
			Rotor current = new Rotor(controller, motor);

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
	public IMyShipController controller;

	public Vector3D localForwardDir = new Vector3D(0, 0, -1);
	// this should be the forward direction local to the rotor head
	public Vector3D headOffset = new Vector3D(0, 0, -1);

	public string theString;


	public Rotor(IMyShipController controller, IMyMotorStator rotor) {
		this.controller = controller;
		this.theBlock = rotor;
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
		float maxRotorRPM = theBlock.GetMaximum<float>("Velocity");
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