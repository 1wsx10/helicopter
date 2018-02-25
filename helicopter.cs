public void Main(string argument) {
	writeBool = false;
	// setup control module
	Dictionary<string, object> inputs;
	try {
		inputs = Me.GetValue<Dictionary<string, object>>("ControlModule.Inputs");
	} catch(Exception exception) {
		Echo("Control Module not installed");
		Echo("Go install the mod linked on this scripts workshop page");
		Echo("Exception: " + exception);
		return;
	}

	// controls
	bool c = inputs.ContainsKey("c.crouch"); // find a specific key
	bool space = inputs.ContainsKey("c.jump"); // find a specific key
	bool w = inputs.ContainsKey("c.forward"); // find a specific key
	bool s = inputs.ContainsKey("c.backward"); // find a specific key
	bool a = inputs.ContainsKey("c.strafeleft"); // find a specific key
	bool d = inputs.ContainsKey("c.straferight"); // find a specific key
	bool q = inputs.ContainsKey("c.rollleft"); // find a specific key
	bool e = inputs.ContainsKey("c.rollright"); // find a specific key

	IMyShipController controller = (IMyShipController)GridTerminalSystem.GetBlockWithName("Cockpit");

	// get rotors
	Rotor mBA = new Rotor(controller, (IMyMotorStator)GridTerminalSystem.GetBlockWithName("MRotor A"));
	Rotor mBB = new Rotor(controller, (IMyMotorStator)GridTerminalSystem.GetBlockWithName("MRotor B"));
	Rotor tBA = new Rotor(controller, (IMyMotorStator)GridTerminalSystem.GetBlockWithName("TRotor A"));
	Rotor tBB = new Rotor(controller, (IMyMotorStator)GridTerminalSystem.GetBlockWithName("TRotor B"));

	IMyMotorStator mShaft = (IMyMotorStator)GridTerminalSystem.GetBlockWithName("MRotor");
	IMyMotorStator tShaft = (IMyMotorStator)GridTerminalSystem.GetBlockWithName("TRotor");



	SwashPlate mainSwash = new SwashPlate(controller, mBA, mBB, mShaft);
	SwashPlate antiTrq = new SwashPlate(controller, tBA, tBB, tShaft);



	antiTrq.collective = -0.25f;
	if(d) {
		antiTrq.collective = -0.5f;
	}
	if(a) {
		antiTrq.collective = 0.19f;
	}


	float collective = 0.45f;
	float cyclic = 0.3f;

	mainSwash.collective = 0.1f;
	if(space) {
		mainSwash.collective = collective;
	}
	if(c) {
		mainSwash.collective = -collective;
	}
	if(w) {
		mainSwash.cyclicR = cyclic;
	}
	if(s) {
		mainSwash.cyclicR = -cyclic;
	}
	if(e) {
		mainSwash.cyclicF = cyclic;
	}
	if(q) {
		mainSwash.cyclicF = -cyclic;
	}


	// roll stability



	// mouse control
	mainSwash.cyclicR += controller.RotationIndicator.X * 0.01f;
	antiTrq.collective += controller.RotationIndicator.Y * -0.01f;


	mainSwash.go();
	antiTrq.go();



	write(mainSwash.theStr);
	write(antiTrq.theStr);
	Echo(mainSwash.theStr);
	write(mBA.theString);
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

public class SwashPlate {

	// physical parts
	public Rotor rA;
	public Rotor rB;
	public IMyShipController controller;
	public IMyMotorStator rotorShaft;

	// vectors
	public Vector3D aVec;//forward direction of that rotor
	public Vector3D bVec;
	public MatrixD contMat;
	public MatrixD shafMat;
	public MatrixD shafMatI;

	// controls (-1 to 1)
	public float collective;// positive = accelerate up
	public float cyclicF;// positive = pitch down
	public float cyclicR;// positive = roll right

	public string theStr = "";

	public SwashPlate(IMyShipController controller, Rotor rA, Rotor rB, IMyMotorStator rotorShaft) {
		this.rA = rA;
		this.rB = rB;
		this.controller = controller;
		this.rotorShaft = rotorShaft;
		this.collective = 0;
		this.cyclicF = 0;
		this.cyclicR = 0;

		contMat = controller.WorldMatrix;
		shafMat = rotorShaft.WorldMatrix;
		shafMatI = MatrixD.Invert(shafMat);

		aVec = baseVector(aVec, rA.axis);
		bVec = baseVector(bVec, rB.axis);
	}

	// gets a vector in the direction the rotor should be moving
	public Vector3D baseVector(Vector3D vec, Vector3D rotAxis) {
		return Vector3D.Normalize(Vector3D.Cross(shafMat.Up, rotAxis));
	}

	public void go() {
		theStr = "collective: " + collective;
		theStr += "\ncyclicF: " + cyclicF;
		theStr += "\ncyclicR: " + cyclicR;
		Vector3D conRi = Vector3D.Transform(contMat.Right, shafMatI);
		Vector3D conFo = Vector3D.Transform(contMat.Forward, shafMatI);
		// Vector3D.Transform(contMat.Right, shafMatI)
		// Vector3D.Transform(contMat.Forward, shafMatI)
		rA.desiredVec = aVec + collective * shafMat.Up + Vector3D.Dot(aVec, shafMat.Right) * shafMat.Up * cyclicF + Vector3D.Dot(aVec, shafMat.Forward) * shafMat.Up * cyclicR;
		rB.desiredVec = bVec + collective * shafMat.Up + Vector3D.Dot(bVec, shafMat.Right) * shafMat.Up * cyclicF + Vector3D.Dot(bVec, shafMat.Forward) * shafMat.Up * cyclicR;
		theStr += "\naVec: " + aVec.ToString("0.0");
		theStr += "\naVec: " + rA.desiredVec.ToString("0.0");
		rA.doTrig2();
		rB.doTrig2();
	}
}

public class Rotor {
	public IMyMotorStator rotor;
	// world space
	public Vector3D desiredVec;
	public float angle;
	public string theString;
	public Vector3D axis;
	public float finalAngle;
	public IMyShipController controller;
	public int offset;

	public Rotor(IMyShipController controller) {// make sure you get axis if you need it
		this.controller = controller;
	}

	public Rotor(IMyShipController controller, IMyMotorStator rotor) {
		this.controller = controller;
		this.rotor = rotor;
		getAxis();
	}

	// sets the rotor offset from the name
	public void checkRotName() {
		int nameA = rotor.CustomName.IndexOf("%(");
		int nameB = rotor.CustomName.IndexOf(")");
		if(nameA != -1 && nameB > nameA) {
			string offsetStr = rotor.CustomName.Substring(nameA + 2, (nameB - nameA) - 2);
			if (Int32.TryParse(offsetStr, out offset)) {
				// theString = "\noffset for " + rotor.CustomName + " set to " + offset + " degrees";
			}
			else {
				// theString = "\nERROR: rotor offset could not be parsed";
			}
		}
	}

	// gets the rotor
	public void getAxis() {
		theString = "\n" + rotor.CustomName + ": ";
		MatrixD rotorMatrix = rotor.WorldMatrix;

		axis = rotorMatrix.Up;// axis is in world space
		if(axis.Length() != 1) {// TODO: remove this and check if it still works
			axis.Normalize();
		}
	}

	// sets the angle to be in the direction of the vector
	public void doTrig() {
		// transform by matrix to go to world space
		// transform by matrixI to go to local space
		MatrixD rotorMatrix = rotor.WorldMatrix;
		MatrixD rotorMatrixI = MatrixD.Invert(rotorMatrix);

		// transpose is cheaper inverse, but only use it with orientation. no translation
		// Matrix.Transpose(ref rotorMatrix, out rotorMatrix);

		// this.desiredVec.Normalize();

		// turn desiredVec from world space to rotor local space
		theString += "\ndesiredVec: \n" + desiredVec.ToString("0.0");
		desiredVec = Vector3D.Transform(desiredVec, rotorMatrixI);
		theString += "\ndesiredVec: \n" + desiredVec.ToString("0.0");
		desiredVec.Normalize();
		theString += "\ndesiredVec: \n" + desiredVec.ToString("0.0");
		// theString += "\nlength: " + desiredVec.Length();

		this.angle = (float)Math.Atan(desiredVec.Z/desiredVec.X);
		if(desiredVec.X > 0) {
			if(desiredVec.Z > 0) {
				// x+ z+
				theString += "\nx+ z+";
				this.angle = -(float)(2*Math.PI - this.angle);
			} else {
				// x+ z-
				theString += "\nx+ z-";
			}
		} else {
			if(desiredVec.Z > 0) {
				// x- z+
				theString += "\nx- z+";
				this.angle = (float)(Math.PI + this.angle);
			} else {
				// x- z-
				theString += "\nx- z-";
				this.angle = (float)(Math.PI + this.angle);
			}
		}
		theString += "\nangle: " + Math.Round(180*this.angle/Math.PI, 1);

		setPos(angle + (float)(offset * Math.PI / 180));
	}

	public double angleBetweenCos(Vector3D a, Vector3D b) {
		double dot = Vector3D.Dot(a, b);
		double Length = a.Length() * b.Length();
		return dot/Length;
	}

	public void doTrig2() {
		desiredVec = Vector3D.Reject(desiredVec, axis);

		// angle between vectors
		float angle = (float)Math.Acos(angleBetweenCos(rotor.WorldMatrix.Forward, desiredVec));

		if(Math.Acos(angleBetweenCos(rotor.WorldMatrix.Left, desiredVec)) > Math.PI/2) {
			angle = (float)(2*Math.PI - angle);
		}

		setPos(angle + (float)(offset * Math.PI / 180));
	}

	float cutAngle(float angle) {
		while(angle > Math.PI) {
			angle -= 2*(float)Math.PI;
		}
		while(angle < -Math.PI) {
			angle += 2*(float)Math.PI;
		}
		// theString += "\nnew Angle: " + Math.Round(180*angle/Math.PI, 1);
		return angle;
	}

	void setPos(float x)//no rotor limits here
	{
		theString += "\nbefore cut: " + (float)Math.Round(180 * x / Math.PI);
		x = cutAngle(x);
		float velocity = 60;
		// theString += "\nSetting angle to:\n" + (x * 180/Math.PI);
		float x2 = cutAngle(rotor.Angle);
		theString += "\nFinal Angle: " + (float)Math.Round(180 * x / Math.PI);
		if(Math.Abs(x - x2) < Math.PI) {
			if(x2 < x) {//dont cross origin
				rotor.SetValue<float>("Velocity", velocity * Math.Abs(x - x2));
			}
			else {
				rotor.SetValue<float>("Velocity", -velocity * Math.Abs(x - x2));
			}
		}
		else {
			//cross origin
			if(x2 < x) {
				rotor.SetValue<float>("Velocity", -velocity * Math.Abs(x - x2));
			}
			else {
				rotor.SetValue<float>("Velocity", velocity * Math.Abs(x - x2));
			}
		}
	}
}