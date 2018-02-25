public Program() {
	Runtime.UpdateFrequency = UpdateFrequency.Update1;
}

public string log = "";

int counter = 0;
public void Main(string argument) {
	writeBool = false;

	Echo($"{counter++}");

	Echo($"{Runtime.TimeSinceLastRun}");
	if(Runtime.TimeSinceLastRun.Milliseconds > 16) {
		log += $"Time was greater than 0.016: \n\t{Runtime.TimeSinceLastRun.Milliseconds}ms";
	}

	Echo(log);

	IMyShipController controller = (IMyShipController)GridTerminalSystem.GetBlockWithName("Cockpit");

	// get rotors
	IMyMotorStator mShaft = (IMyMotorStator)GridTerminalSystem.GetBlockWithName("MRotor");
	IMyMotorStator tShaft = (IMyMotorStator)GridTerminalSystem.GetBlockWithName("TRotor");

	IMyMotorStator[] mainSwashRotors = new IMyMotorStator[] {
		(IMyMotorStator)GridTerminalSystem.GetBlockWithName("MRotor A"),
		(IMyMotorStator)GridTerminalSystem.GetBlockWithName("MRotor B")
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

	antiTrqCont.collective = 0.25f;
	// if(d) {
	// 	antiTrqCont.collective = -0.5f;
	// }
	// if(a) {
	// 	antiTrqCont.collective = 0.19f;
	// }


	float collectiveDefault = 0.45f;
	float cyclicDefault = 0.3f;

	mainSwashCont.collective = 0.1f;

	float rollTrim = -0.008f;

	antiTrqCont.collective += controller.MoveIndicator.X * collectiveDefault;
	mainSwashCont.collective += controller.MoveIndicator.Y * collectiveDefault;
	mainSwashCont.cyclicR += controller.MoveIndicator.Z * cyclicDefault;
	mainSwashCont.cyclicF += controller.RollIndicator * 0.3f * cyclicDefault + rollTrim;

	mainSwashCont.collective *= -1;
	// mainSwashCont.cyclicR *= -1;
	// mainSwashCont.cyclicF *= -1;

	// mouse control
	mainSwashCont.cyclicR += controller.RotationIndicator.X * -0.01f;
	antiTrqCont.collective += controller.RotationIndicator.Y * 0.01f;


	// temp, swap these around
	float temp = mainSwashCont.cyclicR;
	mainSwashCont.cyclicR = mainSwashCont.cyclicF;
	mainSwashCont.cyclicF = temp;



	// TODO: roll stability


	mainSwash.go(mainSwashCont);
	antiTrq.go(antiTrqCont);


	// text
	write(mainSwash.theStr);
	write(antiTrq.theStr);
	Echo(mainSwash.theStr);
	write(mainSwash.blades[0].theString);
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