// public Program() {
// 	Runtime.UpdateFrequency = UpdateFrequency.Update1;
// }

public string log = "";

public const bool swapPitchAndRoll = false;

public const bool enablePID = true;
public float idecay = 1f;

Kinematics km;

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

	IMyShipController controller = (IMyShipController)GridTerminalSystem.GetBlockWithName("Cockpit Forward");


	string output ="";
	if(km == null)
	{
		km = new Kinematics((IMyEntity)controller, null);
	}
	else
	{
		km.Update(Runtime.TimeSinceLastRun.TotalSeconds);

		output += string.Format("Velocity (Linear) [m/s]\n{0}\nVelocity (Angular) [rad/s]\n{1}\n",
				km.VelocityLinearCurrent.ToString("0.000\n"),
				km.VelocityAngularCurrent.ToString("0.000\n")
			);
		output += string.Format("Acceleration (Linear) [m/s²]\n{0}\nAcceleration (Angular) [rad/s²]\n{1}\n",
				km.AccelerationLinearCurrent.ToString("0.000\n"),
				km.AccelerationAngularCurrent.ToString("0.000\n")
			);
		output += string.Format("Jerk (Linear) [m/s³]\n{0}\nJerk (Angular) [rad/s³]\n{1}\n",
				km.JerkLinearCurrent.ToString("0.000\n"),
				km.JerkAngularCurrent.ToString("0.000\n")
			);
	}
	write("Kinematics: \n" + output);



	// get rotors
	IMyMotorStator mShaft = (IMyMotorStator)GridTerminalSystem.GetBlockWithName("MRotor");
	IMyMotorStator tShaft = (IMyMotorStator)GridTerminalSystem.GetBlockWithName("TRotor");

	IMyMotorStator[] mainSwashRotors = new IMyMotorStator[] {
		(IMyMotorStator)GridTerminalSystem.GetBlockWithName("MRotor A"),
		(IMyMotorStator)GridTerminalSystem.GetBlockWithName("MRotor B"),
		(IMyMotorStator)GridTerminalSystem.GetBlockWithName("MRotor C"),
		(IMyMotorStator)GridTerminalSystem.GetBlockWithName("MRotor D")
	};

	IMyMotorStator[] antiTrqRotors = new IMyMotorStator[] {
		(IMyMotorStator)GridTerminalSystem.GetBlockWithName("TRotor A"),
		(IMyMotorStator)GridTerminalSystem.GetBlockWithName("TRotor B")
	};


	// construct swashplates
	SwashPlate mainSwash = new SwashPlate(controller, mainSwashRotors, mShaft);
	SwashPlate antiTrq = new SwashPlate(controller, antiTrqRotors, tShaft);

	// setup controls
	Controls mainSwashCont = new Controls();
	Controls antiTrqCont = new Controls();

	antiTrqCont.collective = 0.1f;


	float collectiveDefault = 0.15f;
	float cyclicDefault = 0.3f;

	mainSwashCont.collective = 0.1f;

	float rollTrim = 0.005f;

	antiTrqCont.collective += controller.MoveIndicator.X * collectiveDefault;
	mainSwashCont.collective += controller.MoveIndicator.Y * collectiveDefault;
	mainSwashCont.cyclicR += controller.MoveIndicator.Z * cyclicDefault;
	mainSwashCont.cyclicF += controller.RollIndicator * -0.3f * cyclicDefault + rollTrim;

	mainSwashCont.collective *= -1;
	// mainSwashCont.cyclicR *= -1;
	// mainSwashCont.cyclicF *= -1;

	// mouse control
	mainSwashCont.cyclicR += controller.RotationIndicator.X * -0.01f;
	antiTrqCont.collective += controller.RotationIndicator.Y * 0.01f;


	// swap these around for multiplayer
	if(swapPitchAndRoll) {
		float temp = mainSwashCont.cyclicR;
		mainSwashCont.cyclicR = mainSwashCont.cyclicF;
		mainSwashCont.cyclicF = temp;
	}



	// TODO: roll stability


	mainSwash.go(mainSwashCont);
	antiTrq.go(antiTrqCont);


	// text
	// write(mainSwash.theStr);
	// write(antiTrq.theStr);
	Echo(mainSwash.theStr);
	// write(mainSwash.blades[0].theString);
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
		double deltaT = prog.Runtime.TimeSinceLastRun.TotalMilliseconds;
		deltaT = (deltaT == 0 ? 1 : deltaT);

		integral *= prog.idecay;
		integral += error/deltaT;
		double derivative = (error - lasterror) / deltaT;
		lasterror = error;

		// return error * pmul + integral * imul + derivative * dmul;
		if(!enablePID) {
			return error;
		}
		return error * pmul + integral * imul + -1 * derivative * dmul;
	}
}

public struct Controls {
	public float collective;
	public float cyclicF;
	public float cyclicR;
}

public class SwashPlate {

	// physical parts
	public List<Rotor> blades;
	public IMyShipController controller;
	public IMyMotorStator rotorShaft;

	public string theStr = "";

	public SwashPlate(IMyShipController controller, IMyMotorStator[] blades, IMyMotorStator rotorShaft) {
		this.controller = controller;
		this.rotorShaft = rotorShaft;

		this.blades = new List<Rotor>();
		foreach(IMyMotorStator motor in blades) {
			Rotor current = new Rotor(controller, motor);

			current.setForwardDir(Vector3D.Cross(rotorShaft.WorldMatrix.Up, motor.WorldMatrix.Up));
			current.headOffset = current.localForwardDir; //no offset

			this.blades.Add(current);
		}
	}

	// should keep controls between -1 and 1
	public void go(Controls cont) {
		theStr = "collective: " + cont.collective;
		theStr += "\ncyclicF: " + cont.cyclicF;
		theStr += "\ncyclicR: " + cont.cyclicR;

		bool first = true;
		foreach(Rotor blade in blades) {
			Vector3D vec =
				// collective
				blade.getForwardDir() + rotorShaft.WorldMatrix.Up * cont.collective +
				// cyclic forward
				Vector3D.Dot(blade.getForwardDir(), rotorShaft.WorldMatrix.Right) * rotorShaft.WorldMatrix.Up * cont.cyclicF +
				// cyclic right
				Vector3D.Dot(blade.getForwardDir(), rotorShaft.WorldMatrix.Forward) * rotorShaft.WorldMatrix.Up * cont.cyclicR;

			blade.setFromVec(vec);

			if(first) {
				first = false;

				theStr += "\nfirst basevec: " + blade.localForwardDir.ToString("0.0");
				theStr += "\nfirst desired: " + vec.ToString("0.0");
			}
		}
	}
}

public class Rotor {

	public IMyMotorStator theBlock;
	public IMyShipController controller;

	public Vector3D localForwardDir;
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