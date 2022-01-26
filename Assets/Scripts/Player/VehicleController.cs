using Helpers;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class VehicleController : MonoBehaviour
{
	[Header("Components")]
	public Transform MainThruster;
	public Transform CameraFollowPoint;
	public RectTransform Crosshair;
	public Transform[] ChainGunBulletSpawnPoints;
	public Transform[] MissileSpawnPoints;
	public Transform[] RailgunBeamSpawnPoints;
	public Transform MissileProjectile;
	public Transform GroundSensorContainer;
	public Transform GroundCorrectionSensor;
	public Collider Hurtbox;

	public ChaingunWeapon Chaingun;
	public SmartMissileWeapon SmartMissiles;
	public RailgunWeapon Railgun;

	public enum WeaponType
	{
		Chaingun,
		Missiles,
		Railgun
	}

	[Header("Player Properties")]
	public int MaxHealth = 800;
	[ReadOnly] public int CurrentHealth;

	public float MaxJumpBoost = 100;
	[ReadOnly] public float CurrentJumpBoost;
	public float JumpBoostBurnRate = 60;
	public float JumpBoostRechargeRate = 20;
	[Range(0, 1)] public float JumpBoostFullyBurnRechargePercentage = 0.3f;

	//public float MaxDashBoost = 100;
	[ReadOnly] public float CurrentDashBoostForce;
	//public float DashBoostRechargeRate = 25;
	//[Range(0, 1)] public float DashBoostFullyBurnRechargePercentage = 0.3f;

	[Header("Forces")]
	public float horizontalAcceleration = 1.15f;
	public float groundDetectionHeight = 3.6f; //Necessary to preempt the landing for the main thruster
	public float hoverForce = 2.8f;
	public float hoverHeight = 2.8f;
	public float groundCorrectionHoverForce = 12f;
	public float groundCorrectionHoverHeight = 0.9f;
	public float crashForce = 9f;
	public float minBoostForce = 0.25f;
	public float maxBoostForce = 1.25f;
	public float DashBoostIncreaseRate = 0.25f;
	public float jumpForce = 1.6f;
	public VectorPID angularVelocityController;
	public VectorPID headingController;
	public VectorPID upAngularVelocityController;
	public VectorPID upHeadingGroundedController;
	public VectorPID upHeadingInAirController;

	[Header("Rotation")]
	public float hoverRotationCorrectionLerpStrengthOnGround = 6.8f; //This is what will be used to turn toward the direction the camera is facing
	public float hoverRotationCorectionLerpStrengthInAir = 0.4f; //This is what will be used to turn toward the direction the camera is facing
	public float turnLerpStrength = 2.2f; //This is what will be used to turn toward the direction the camera is facing

	[Header("Misc")]
	public List<Collider> IgnoredColliders; //TODO: Move to PlayerCamera.

	public Vector3 CentreOfMass { get { return Hurtbox.bounds.center; } }
	public WeaponType EquippedWeaponType { get { return equippedWeaponIndex; } }
	public EquippableWeapon EquippedWeapon { get { return weapons[(int)equippedWeaponIndex]; } }

	public bool Jumping { get { return inputJump && !jumpBoostFullyBurned; } }
	public bool Dashing { get { return inputSprint; } }
	public float Speed { get { return new Vector3(vehicleRigidbody.velocity.x, 0, vehicleRigidbody.velocity.z).magnitude * 3.6f; } }

	private Transform[] GroundSensors = new Transform[0];
	private Rigidbody vehicleRigidbody;
	private PlayerCamera playerCamera;
	private VectorPIDInstance angularVelocityPID;
	private VectorPIDInstance headingPID;
	private VectorPIDInstance upAngularVelocityPID;
	private VectorPIDInstance upHeadingGroundedPID;
	private VectorPIDInstance upHeadingInAirPID;

	private EquippableWeapon[] weapons;
	private WeaponType equippedWeaponIndex;

	public Vector3[] FlankPoints = {
		new Vector3(0,0,1).normalized,
		new Vector3(1,0,1).normalized,
		new Vector3(1,0,0).normalized,
		new Vector3(1,0,-1).normalized,
		new Vector3(0,0,-1).normalized,
		new Vector3(-1,0,-1).normalized,
		new Vector3(-1,0,0).normalized,
		new Vector3(-1,0,1).normalized
	};

	private Vector3 averageGroundNormal = new Vector3();
	private bool isGrounded = false;
	private Vector3 planarThrustDirection = new Vector3();

	private Vector3 inputMove;
	private Vector2 inputLook;
	private bool inputSprint;
	private bool inputSprintConsumed;
	private bool inputJump;
	private bool inputJumpConsumed;
	private bool inputFire;
	private bool inputFireConsumed;
	private bool inputAltFire;
	private bool inputAltFireConsumed;
	private bool inputWeaponSwitch;
	private bool inputWeaponSwitchConsumed;

	private bool jumpBoostFullyBurned;

	private void Awake()
	{
		this.vehicleRigidbody = GetComponent<Rigidbody>();
		this.playerCamera = GameObject.FindGameObjectWithTag(Helpers.Tags.PlayerCamera).GetComponent<PlayerCamera>();

		angularVelocityPID = angularVelocityController.Instantiate();
		headingPID = headingController.Instantiate();
		upAngularVelocityPID = upAngularVelocityController.Instantiate();
		upHeadingGroundedPID = upHeadingGroundedController.Instantiate();
		upHeadingInAirPID = upHeadingInAirController.Instantiate();


		var thrusters = this.GroundSensorContainer.GetComponentsInChildren<Transform>();
		this.GroundSensors = new Transform[thrusters.Length];
		for (int i = 0; i < thrusters.Length; i++)
		{
			this.GroundSensors[i] = thrusters[i];
		}
	}

	private void Start()
	{
		Cursor.lockState = CursorLockMode.Locked;

		this.weapons = new EquippableWeapon[]
		{
			Chaingun,
			SmartMissiles,
			Railgun
		};
		this.equippedWeaponIndex = 0;
		foreach (var weapon in weapons)
		{
			weapon.Initialise();
		}

		InitialisePlayerProperties();
		InitialiseWeaponProperties();

		EquippedWeapon.SwitchWeapon();
	}

	private void Update()
	{
		this.isGrounded = false;

		this.averageGroundNormal = CalculateGroundNormalFromThrusters();
		if (this.averageGroundNormal.magnitude > 0)
		{
			this.isGrounded = true;
		}

		this.planarThrustDirection = new Vector3();
		var rotateToNormalQuaternion = Quaternion.LookRotation(Vector3.Cross(this.isGrounded ? this.averageGroundNormal : Vector3.up, transform.right), -this.averageGroundNormal);
		var cameraPlanarMovementForwardRotation = Quaternion.Euler(rotateToNormalQuaternion.eulerAngles.x, this.playerCamera.transform.rotation.eulerAngles.y, rotateToNormalQuaternion.eulerAngles.z);

		if (Dashing)
		{
			this.planarThrustDirection = (cameraPlanarMovementForwardRotation * new Vector3(Mathf.Clamp(this.inputMove.x, -0.5f, 0.5f), 0, 1).normalized);
		}
		else if (Mathf.Abs(this.inputMove.magnitude) > 0)
		{
			this.planarThrustDirection = (cameraPlanarMovementForwardRotation * this.inputMove.normalized);
		}

		if (Jumping)
		{
			CurrentJumpBoost = Mathf.Clamp(CurrentJumpBoost - JumpBoostBurnRate * Time.deltaTime, 0, MaxJumpBoost);

			if (CurrentJumpBoost == 0)
				jumpBoostFullyBurned = true;
		}
		else
		{
			CurrentJumpBoost = Mathf.Clamp(CurrentJumpBoost + JumpBoostRechargeRate * Time.deltaTime, 0, MaxJumpBoost);

			if (jumpBoostFullyBurned && CurrentJumpBoost >= JumpBoostFullyBurnRechargePercentage * MaxJumpBoost)
				jumpBoostFullyBurned = false;
		}

		if (Dashing)
		{
			CurrentDashBoostForce = Mathf.Clamp(CurrentDashBoostForce + DashBoostIncreaseRate * Time.deltaTime, minBoostForce, maxBoostForce);
		}
		else
		{
			CurrentDashBoostForce = Mathf.Clamp(minBoostForce, minBoostForce, maxBoostForce);
		}

		foreach (var weapon in weapons)
		{
			if (weapon.Type == equippedWeaponIndex)
				weapon.WeaponUpdate(this.inputFire);
			else
				weapon.WeaponUpdate(false);
		}
	}

	private void LateUpdate()
	{
		HandleCameraInput();
	}

	private void FixedUpdate()
	{
		RaycastHit hit;
		if (!this.isGrounded && Physics.Raycast(this.GroundCorrectionSensor.position, -this.GroundCorrectionSensor.up, out hit, this.groundCorrectionHoverHeight, Helpers.Masks.Ground) ||
			(this.isGrounded && averageGroundNormal == Vector3.zero))
		{
			Debug.Log($"Trying to push the player ship off the ground");
			this.vehicleRigidbody.AddForce(this.GroundCorrectionSensor.up * this.groundCorrectionHoverForce, ForceMode.VelocityChange);
		}
		else if (Physics.Raycast(this.MainThruster.position, this.averageGroundNormal, out hit, this.hoverHeight, Helpers.Masks.Ground))
		{
			this.vehicleRigidbody.AddForce(-this.averageGroundNormal * this.hoverForce * (1f - (hit.distance / this.hoverHeight)), ForceMode.VelocityChange);
		}

		if (Mathf.Abs(this.planarThrustDirection.magnitude) > 0)
		{
			this.vehicleRigidbody.AddForce(this.planarThrustDirection * this.horizontalAcceleration, ForceMode.VelocityChange);

			if (Dashing)
				this.vehicleRigidbody.AddForce(this.planarThrustDirection * this.CurrentDashBoostForce, ForceMode.VelocityChange);
		}

		if (Jumping)
			this.vehicleRigidbody.AddForce(Vector3.up * this.jumpForce, ForceMode.VelocityChange);

		ApplyRotationalTorqueWithPIDs();
	}

	public void ReceiveDamage(int damagePoints)
	{
		CurrentHealth = Mathf.Clamp(CurrentHealth - damagePoints, 0, MaxHealth);

		if (this.CurrentHealth <= 0)
		{
			//TODO: Blow up ship.
			Debug.Log("Player died");
		}
	}

	public Vector3[] GetPositionsToFlankPosition(Vector3 startPosition, Vector3 targetPosition, float minFlankDistance, ref NavMeshPath navPathToDestination)
	{
		Vector3[] adjustedPositions = {
			transform.position + FlankPoints[0] * minFlankDistance,
			transform.position + FlankPoints[1] * minFlankDistance,
			transform.position + FlankPoints[2] * minFlankDistance,
			transform.position + FlankPoints[3] * minFlankDistance,
			transform.position + FlankPoints[4] * minFlankDistance,
			transform.position + FlankPoints[5] * minFlankDistance,
			transform.position + FlankPoints[6] * minFlankDistance,
			transform.position + FlankPoints[7] * minFlankDistance,
		};

		//foreach (var position in adjustedPositions)
		//{
		//	Debug.DrawLine(transform.position, position, Color.cyan, 3f);
		//}

		List<Vector3> pathToDestination = new List<Vector3>();
		Vector3 latestPosition = startPosition;
		int loopCount = 0;

		while (latestPosition != targetPosition && loopCount <= adjustedPositions.Length)
		{
			bool updatedPosition = false;

			//Look through next closest flank position.
			var latestDistanceToFlank = float.MaxValue;
			var distanceToDestinationWithLatestPosition = Vector3.Distance(latestPosition, targetPosition);
			var newLatestPosition = latestPosition;

			foreach (var flankPosition in adjustedPositions)
			{
				var differentFlankFromLatestPosition = Vector3.Distance(latestPosition, flankPosition) > 0; //Don't stop where you are
				var differentFlankFromNewLatestPosition = Vector3.Distance(newLatestPosition, flankPosition) > 0; //Don't go in circles
				var distanceToFlank = Vector3.Distance(latestPosition, flankPosition); //Must be closest flank to the new latest position
				var distanceFromFlankToDestination = Vector3.Distance(flankPosition, targetPosition); //Must also be closest to the destination

				if (differentFlankFromLatestPosition && differentFlankFromNewLatestPosition &&
					distanceToFlank < latestDistanceToFlank &&
					distanceFromFlankToDestination < distanceToDestinationWithLatestPosition)
				{
					newLatestPosition = flankPosition;
					latestDistanceToFlank = distanceToFlank;
					updatedPosition = true;
				}
			}

			latestPosition = updatedPosition ? newLatestPosition : targetPosition;
			pathToDestination.Add(latestPosition);
			loopCount++;
		}

		//Debug.Log($"Found {pathToDestination.Count} positions to goal. Traversing through with NavMeshPath.");
		//Debug.Log($"Path: {string.Join(",", pathToDestination.ToArray())}");

		var lastPosition = startPosition;
		IEnumerable<Vector3> totalCorners = new Vector3[] { };

		foreach (var position in pathToDestination)
		{
			NavMesh.CalculatePath(lastPosition, position, NavMesh.AllAreas, navPathToDestination);
			totalCorners = totalCorners.Concat(navPathToDestination.corners);
			lastPosition = position;
		}

		//Debug.Log($"Total corners found: {totalCorners.Count()}");
		//Debug.Log($"Corners: {string.Join(",", totalCorners.ToArray())}");

		return totalCorners.ToArray();
	}

	private void InitialisePlayerProperties()
	{
		this.CurrentHealth = this.MaxHealth;
		this.CurrentJumpBoost = this.MaxJumpBoost;
	}

	private void InitialiseWeaponProperties()
	{
		List<ParticleSystem> chaingunBulletTrailSystemList = new List<ParticleSystem>();
		foreach (var spawnPoint in ChainGunBulletSpawnPoints)
		{
			chaingunBulletTrailSystemList.Add(spawnPoint.GetComponentInChildren<ParticleSystem>());
		}
		var chaingunBulletTrailSystems = chaingunBulletTrailSystemList.ToArray();

		List<ParticleSystem> railgunBeamTrailSystemList = new List<ParticleSystem>();
		foreach (var spawnPoint in RailgunBeamSpawnPoints)
		{
			railgunBeamTrailSystemList.Add(spawnPoint.GetComponentInChildren<ParticleSystem>());
		}
		var railgunBeamTrailSystems = railgunBeamTrailSystemList.ToArray();

		//TODO: Refactor the weapons from being scriptable objects to being prefabs containing the weapon model, the muzzle flash system, an audio player, bullet trails and spawn points.
		//All of this needs to be a single component now
		Chaingun.PlayerCamera = playerCamera;
		Chaingun.Crosshair = Crosshair;
		//Chaingun.MuzzleFlashSystems = chaingunMuzzleFlashSystems;
		Chaingun.BulletTrailSystems = chaingunBulletTrailSystems;
		Chaingun.BulletSpawnPoints = ChainGunBulletSpawnPoints;

		SmartMissiles.PlayerCamera = playerCamera;
		SmartMissiles.Crosshair = Crosshair;
		//SmartMissiles.MuzzleFlashSystems = missileMuzzleFlashSystems;
		SmartMissiles.BulletSpawnPoints = MissileSpawnPoints;

		Railgun.PlayerCamera = playerCamera;
		Railgun.Crosshair = Crosshair;
		//Railgun.MuzzleFlashSystems = missileMuzzleFlashSystems;
		Railgun.BulletTrailSystems = railgunBeamTrailSystems;
		Railgun.BulletSpawnPoints = RailgunBeamSpawnPoints;
		Railgun.Crosshair = Crosshair;
	}

	private Vector3 CalculateGroundNormalFromThrusters()
	{
		RaycastHit hit;
		Vector3 groundNormal = Vector3.zero;
		List<Vector3> thrusterGroundNormals = new List<Vector3>();

		for (int i = 0; i < this.GroundSensors.Length; i++)
		{
			var thruster = this.GroundSensors[i];
			var thrusterUp = transform.up;

			if (Physics.Raycast(thruster.position, -thrusterUp, out hit, this.groundDetectionHeight, Helpers.Masks.Ground))
			{
				thrusterGroundNormals.Add(-hit.normal);
			}
		}

		if (thrusterGroundNormals.Count == 0)
			return Vector3.zero;

		foreach (var thrusterGroundNormal in thrusterGroundNormals)
			groundNormal += thrusterGroundNormal.normalized;

		return groundNormal.normalized;
	}

	private void ApplyRotationalTorqueWithPIDs()
	{
		//Angular velocity
		var angularVelocityError = vehicleRigidbody.angularVelocity * -1;
		//Debug.DrawRay(transform.position, vehicleRigidbody.angularVelocity * 10, Color.black);
		var angularVelocityCorrection = angularVelocityPID.UpdatePID(angularVelocityError, Time.fixedDeltaTime);
		//Debug.DrawRay(transform.position, angularVelocityCorrection, Color.green);
		vehicleRigidbody.AddTorque(angularVelocityCorrection, ForceMode.Impulse);

		//Heading direction velocity
		var desiredHeading = Vector3.ProjectOnPlane(playerCamera.transform.forward, this.isGrounded ? -averageGroundNormal : Vector3.up).normalized;
		//Debug.DrawRay(transform.position, desiredHeading, Color.magenta);
		var currentHeading = transform.forward;
		//Debug.DrawRay(transform.position, currentHeading * 15, Color.blue);
		var headingError = Vector3.Cross(currentHeading, desiredHeading); //TODO: Fix Cross going to zero when facing 100% behind us
		var headingCorrection = headingPID.UpdatePID(headingError, Time.fixedDeltaTime);
		vehicleRigidbody.AddTorque(headingCorrection, ForceMode.Acceleration);

		//Angular velocity for upward orientation
		var upAngularVelocityError = vehicleRigidbody.angularVelocity * -1;
		//Debug.DrawRay(transform.position, vehicleRigidbody.angularVelocity * 10, Color.black);
		var upAngularVelocityCorrection = upAngularVelocityPID.UpdatePID(upAngularVelocityError, Time.fixedDeltaTime);
		//Debug.DrawRay(transform.position, upAngularVelocityCorrection, Color.green);
		vehicleRigidbody.AddTorque(upAngularVelocityCorrection, ForceMode.Impulse);

		//Heading direction velocity for upward orientation
		var desiredUpHeading = this.isGrounded ? -averageGroundNormal : Vector3.up;
		//Debug.DrawRay(transform.position, desiredUpHeading, Color.magenta);
		var currentUpHeading = transform.up;
		//Debug.DrawRay(transform.position, currentUpHeading * 15, Color.blue);
		var upHeadingError = Vector3.Cross(currentUpHeading, desiredUpHeading);
		var upHeadingGroundedCorrection = upHeadingGroundedPID.UpdatePID(upHeadingError, Time.fixedDeltaTime);
		var upHeadingInAirCorrection = upHeadingInAirPID.UpdatePID(upHeadingError, Time.fixedDeltaTime);
		vehicleRigidbody.AddTorque(isGrounded ? upHeadingGroundedCorrection : upHeadingInAirCorrection, ForceMode.Acceleration);
	}

	private void HandleCameraInput()
	{
		if (Cursor.lockState != CursorLockMode.Locked)
		{
			this.inputLook = Vector2.zero;
		}

		// Apply inputs to the camera
		this.playerCamera.UpdateWithInput(Time.deltaTime, 1, this.inputLook);
	}
	public void OnMove(InputAction.CallbackContext context)
	{
		Vector2 inputVec = context.ReadValue<Vector2>();
		this.inputMove = new Vector3(inputVec.x, 0, inputVec.y);
	}

	public void OnLook(InputAction.CallbackContext context)
	{
		Vector2 inputVec = context.ReadValue<Vector2>();
		this.inputLook = new Vector2(inputVec.x, inputVec.y);
	}

	public void OnSprint(InputAction.CallbackContext context)
	{
		this.inputSprint = context.ReadValue<float>() > 0;
	}

	public void OnJump(InputAction.CallbackContext context)
	{
		this.inputJump = context.ReadValue<float>() > 0;
	}

	public void OnFire(InputAction.CallbackContext context)
	{
		if (context.performed)
			return;

		Cursor.lockState = CursorLockMode.Locked;
		this.inputFire = context.ReadValue<float>() > 0;
		if (!this.inputFire)
			return;
	}

	public void OnAltFire(InputAction.CallbackContext context)
	{

	}

	public void OnWeaponSwitch(InputAction.CallbackContext context)
	{
		if (context.performed)
			return;

		float inputDirection = context.ReadValue<float>();
		if (inputDirection > 0)
		{
			this.equippedWeaponIndex++;
			if ((int)this.equippedWeaponIndex == this.weapons.Length)
				this.equippedWeaponIndex = 0;

			EquippedWeapon.SwitchWeapon();
			//print("Weapon Switch UP: Equipped Weapon " + this.equippedWeaponIndex);
		}
		else if (inputDirection < 0)
		{
			if (this.equippedWeaponIndex == 0)
				this.equippedWeaponIndex = (WeaponType)this.weapons.Length;
			this.equippedWeaponIndex--;
			EquippedWeapon.SwitchWeapon();
			//print("Weapon Switch DOWN: Equipped Weapon " + this.equippedWeaponIndex);
		}
	}

	public void OnWeaponSwitchUp(InputAction.CallbackContext context)
	{
	}

	public void OnWeaponSwitchDown(InputAction.CallbackContext context)
	{
	}

	private void OnDrawGizmos()
	{
		RaycastHit hit;

		//Average Ground Normal
		//var averageGroundNormal = CalculateGroundNormalFromThrusters();

		if (Physics.Raycast(this.MainThruster.position,
									this.averageGroundNormal, out hit,
									this.hoverHeight,
									Helpers.Masks.Ground))
		{
			Gizmos.color = Color.green;
			Gizmos.DrawLine(this.MainThruster.position, hit.point);
			Gizmos.DrawSphere(hit.point, 0.5f);
		}
		else
		{
			Gizmos.color = Color.cyan;
			Gizmos.DrawLine(this.MainThruster.position,
						   this.MainThruster.position - this.MainThruster.up * this.hoverHeight);
		}

		//Hover Forces
		foreach (var thruster in this.GroundSensors)
		{
			var thrusterUp = transform.up;

			if (Physics.Raycast(thruster.position,
							-thrusterUp, out hit,
							this.groundDetectionHeight,
							Helpers.Masks.Ground))
			{
				Gizmos.color = Color.blue;
				Gizmos.DrawLine(thruster.position, hit.point);
				Gizmos.DrawSphere(hit.point, 0.5f);
			}
			else
			{
				Gizmos.color = Color.red;
				Gizmos.DrawLine(thruster.position,
							   thruster.position - thrusterUp * this.hoverHeight);
			}
		}

		//Forward position based off of ground
		var lineLength = 1f;
		if (!this.isGrounded && Physics.Raycast(this.GroundCorrectionSensor.position,
						-this.GroundCorrectionSensor.transform.up, out hit,
						this.groundDetectionHeight,
						Helpers.Masks.Ground))
		{
			Gizmos.color = Color.yellow;
			Gizmos.DrawLine(this.GroundCorrectionSensor.position, hit.point);
			Gizmos.DrawSphere(hit.point, 0.5f);
		}
		else
		{
			Gizmos.color = Color.magenta;
			Gizmos.DrawLine(this.GroundCorrectionSensor.position,
						   this.GroundCorrectionSensor.position - this.GroundCorrectionSensor.transform.up * this.hoverHeight);
		}

		//Forward position based off of ground
		Gizmos.color = Color.red;
		Gizmos.DrawLine(this.CameraFollowPoint.position,
			this.CameraFollowPoint.position + (this.isGrounded ? this.averageGroundNormal : Vector3.down) * lineLength);
		Gizmos.color = Color.green;
		Gizmos.DrawLine(this.CameraFollowPoint.position,
			this.CameraFollowPoint.position + transform.right * lineLength);
		Gizmos.color = Color.blue;
		Gizmos.DrawLine(this.CameraFollowPoint.position,
			this.CameraFollowPoint.position + (Vector3.Cross(this.isGrounded ? this.averageGroundNormal : Vector3.down, transform.right) * lineLength));

		//Velocity direction
		var velocityPositionOffset = Vector3.up * 2f;
		var velocityGizmoPosition = this.CameraFollowPoint.position + velocityPositionOffset;
		Gizmos.color = Color.white;
		Gizmos.DrawLine(velocityGizmoPosition,
			velocityGizmoPosition + Vector3.up * lineLength);
		Gizmos.color = Color.magenta;
		Gizmos.DrawLine(velocityGizmoPosition,
			velocityGizmoPosition + this.planarThrustDirection * lineLength);
	}

	private void OnCollisionEnter(Collision collision)
	{
		if (collision.gameObject.layer == Helpers.LayerIDs.EnemyHurtbox)
		{
			//Rammed an enemy
		}
		else
		{
			var collisionContactPoint = collision.GetContact(0);
			vehicleRigidbody.AddForce(collisionContactPoint.normal * this.crashForce, ForceMode.VelocityChange);
		}
	}
}
