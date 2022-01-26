using Helpers;
using SensorToolkit;
using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(LineRenderer))]
public class VehicleAIController : AIController
{
	private const float minimumDistanceToTargetPosition = 4f;
	private const float minimumTimeToRecalculatePosition = 0.5f;

	public enum TurretAIState
	{
		Idle,
		Aiming,
		Firing,
		Reloading,
		Caution,
		Dead,
	}

	public enum AIBehaviourType
	{
		Idle,
		Defensive,
		Aggressive,
		Flanker
	}

	[Header("Components")]
	public Transform MainThruster;
	public RangeSensor VehicleRangeSensor;
	public Transform Vehicle;
	public Transform TurretGun;
	public Transform TurretProjectileSpawnPoint;
	public Transform GroundSensorContainer;
	public Transform GroundCorrectionSensor;

	[Header("Debug")]
	public bool DebugNavPath;

	[Header("Forces")]
	public float horizontalAcceleration = 1.15f;
	public float groundDetectionHeight = 3.6f;
	public float hoverForce = 2.8f;
	public float hoverHeight = 2.8f;
	public float groundCorrectionHoverForce = 12f;
	public float groundCorrectionHoverHeight = 0.9f;
	public VectorPID angularVelocityController;
	public VectorPID headingController;
	public VectorPID upAngularVelocityController;
	public VectorPID upHeadingGroundedController;
	public VectorPID upHeadingInAirController;

	[Header("Enemy Properties")]
	[ReadOnly] public Vector3 StartPosition;
	[ReadOnly] public Vector3 StartDirection;
	[ReadOnly] public Quaternion StartRotation;
	[ReadOnly] public TurretAIState TurretState;

	[Header("General Properties")]
	public bool FixedTurret;
	public float TurretRotationSpeed = 3.5f;
	[Range(0, 1)] public float TrackingPrediction = 0f; //TODO: Implement. 0 - Poor tracking. Shoots directly at the player. 1 - Perfect tracking. Shoots exactly at the position the player would be based on the projectile speed.
	public Transform WeaponProjectilePrefab;
	public float WeaponProjectileSpeed = 5f;
	public float WeaponFireRate = 3.5f;

	[Header("AI Behaviours")]
	public AIBehaviourType BehaviourType;
	public AIBehaviourModule AIBehaviours;

	public TurretAIState CurrentTurretState { get; private set; }
	public TurretAIState PreviousTurretState { get; private set; }
	public float TimeEnteredState { get; private set; }
	public float TimeSinceEnteringState { get { return Time.time - TimeEnteredState; } }

	private Transform[] GroundSensors = new Transform[0];
	private Rigidbody vehicleRigidbody;
	private LineRenderer navPathDebugger;
	private VectorPIDInstance angularVelocityPID;
	private VectorPIDInstance headingPID;
	private VectorPIDInstance upAngularVelocityPID;
	private VectorPIDInstance upHeadingGroundedPID;
	private VectorPIDInstance upHeadingInAirPID;

	private bool isDead;
	private bool playerDetected;
	private Vector3 targetPredictionPosition;
	private Quaternion targetPredictionRotation;

	private Vector3? targetMovementDestination;
	private Vector3? targetMovementNextPosition;
	private NavMeshPath navPathToDestination;
	private Vector3[] pathToDestinationCorners;
	private int pathToDestinationCornerIndex;

	private bool HasDestination { get { return targetMovementDestination.HasValue; } }
	private bool HasNextPosition { get { return targetMovementNextPosition.HasValue; } }
	private float timeSinceLastPathSet;

	[ReadOnly] public Vector3 averageGroundNormal = new Vector3();
	[ReadOnly] public bool isGrounded = false;
	private Vector3 planarThrustDirection = new Vector3();

	private Vector3 aiInputMove;
	//private Vector3 aiInputRotation;

	//TODO: Periodically strafe left and right while in combat, using the strafe setting from Behaviour Settings.
	//TODO: Assign Basic Behaviour Module to the Behaviour Types

	protected override void Awake()
	{
		base.Awake();

		angularVelocityPID = angularVelocityController.Instantiate();
		headingPID = headingController.Instantiate();
		upAngularVelocityPID = upAngularVelocityController.Instantiate();
		upHeadingGroundedPID = upHeadingGroundedController.Instantiate();
		upHeadingInAirPID = upHeadingInAirController.Instantiate();

		this.vehicleRigidbody = GetComponent<Rigidbody>();
		this.navPathDebugger = GetComponent<LineRenderer>();
		this.navPathDebugger.enabled = false;

		var thrusters = this.GroundSensorContainer.GetComponentsInChildren<Transform>();
		this.GroundSensors = new Transform[thrusters.Length];
		for (int i = 0; i < thrusters.Length; i++)
		{
			this.GroundSensors[i] = thrusters[i];
		}
	}

	protected override void Start()
	{
		base.Start();

		StartPosition = transform.position;
		StartDirection = transform.forward;
		StartRotation = transform.rotation;
		TransitionToState(TurretAIState.Idle);
	}

	protected override void Update()
	{
		base.Update();

		switch (CurrentTurretState)
		{
			case TurretAIState.Idle:
				RotateGunTowardForwardPosition();

				if (playerDetected)
				{
					TransitionToState(TurretAIState.Aiming);
				}
				break;

			case TurretAIState.Aiming:
				if (!playerDetected)
				{
					TransitionToState(TurretAIState.Idle);
				}
				else
				{
					RotateGunTowardsTarget();

					if (IsGunOnTarget())
					{
						TransitionToState(TurretAIState.Firing);
					}
				}
				//TODO: If turret is facing the tracking direction close enough, fire
				break;

			case TurretAIState.Firing:
				RotateGunTowardsTarget();

				if (true)//FireAnimationComplete())
					TransitionToState(TurretAIState.Reloading);
				break;

			case TurretAIState.Reloading:
				RotateGunTowardsTarget();

				//TODO: Play animations and do stuff. Then transition back to aiming
				if (TimeSinceEnteringState >= WeaponFireRate)
				{
					TransitionToState(TurretAIState.Aiming);
				}
				break;

			case TurretAIState.Caution:

				break;
		}

		this.aiInputMove = new Vector3();
		this.planarThrustDirection = new Vector3();

		if (!isDead)
		{
			this.averageGroundNormal = CalculateGroundNormalFromThrusters();
			if (this.averageGroundNormal.magnitude > 0)
				this.isGrounded = true;
			else
				this.isGrounded = false;

			if (CurrentTurretState != TurretAIState.Idle)
			{
				if ((timeSinceLastPathSet >= minimumTimeToRecalculatePosition &&
						(AIBehaviours.FlankPlayer ||
						Vector3.Distance(transform.position, Player.transform.position) < AIBehaviours.MinimumDistanceToPlayerInCombat ||
						Vector3.Distance(transform.position, Player.transform.position) > AIBehaviours.MaximumDistanceToPlayerInCombat)))
				{
					CalculateDestination();
				}
				else if (HasDestination)
				{
					CalculateNextPositionToDestination();

					if (HasNextPosition)
					{
						//TODO: Make strafing here. Find a way to ignore the Destination and strafe as well, or just shift the destination while keeping it on the NavMesh
						//TODO: Possibly add Player Avoidance here. If we are too close to the player then we should desire to move away.
						//TODO: Possibly also recalculate destination if player is too close and the last recalculate was not too soon.
						this.aiInputMove = transform.position.DirectionTo(targetMovementNextPosition.Value);
						this.planarThrustDirection = AIBehaviours.AlwaysLooksAtPlayerWhenAlert ? aiInputMove : transform.forward;
					}
				}
				else
				{
					if (targetMovementNextPosition != null)
					{
						targetMovementNextPosition = null;
					}

					if (this.timeSinceLastPathSet >= AIBehaviours.WaitTimeBeforeMovingToNewPosition)
						CalculateDestination();
				}

				this.timeSinceLastPathSet = Mathf.Clamp(this.timeSinceLastPathSet + Time.deltaTime, 0, AIBehaviours.WaitTimeBeforeMovingToNewPosition);
			}
		}
	}

	private void CalculateNextPositionToDestination()
	{
		if (Vector3.Distance(transform.position, targetMovementDestination.Value) <= minimumDistanceToTargetPosition)
		{
			targetMovementNextPosition = null;
			targetMovementDestination = null;
		}
		else if (!HasNextPosition || Vector3.Distance(transform.position, targetMovementNextPosition.Value) <= minimumDistanceToTargetPosition)
		{
			if (pathToDestinationCornerIndex < pathToDestinationCorners.Length)
			{
				targetMovementNextPosition = pathToDestinationCorners[pathToDestinationCornerIndex];
				pathToDestinationCornerIndex++;
			}
			else
				targetMovementNextPosition = targetMovementDestination.Value;
		}
	}

	private void CalculateDestination()
	{
		var destination = transform.position;

		if (AIBehaviours.FlankPlayer)
		{
			var playerRear = -Player.transform.forward;
			var randomAngle = UnityEngine.Random.Range(-AIBehaviours.AngleOfDeviationToChooseNewPosition, AIBehaviours.AngleOfDeviationToChooseNewPosition);

			var rotatedDirectionFromPlayerRear = Quaternion.AngleAxis(randomAngle, Vector3.up) * playerRear;
			//Debug.DrawLine(transform.position, Player.transform.position, Color.white, 3f);
			//Debug.DrawRay(Player.transform.position + playerRear * AIBehaviours.MinimumDistanceToPlayerInCombat, Vector3.up * 5f, Color.green, minimumTimeToRecalculatePosition);
			//Debug.DrawRay(Player.transform.position + playerRear * AIBehaviours.MaximumDistanceToPlayerInCombat, Vector3.up * 5f, Color.green, minimumTimeToRecalculatePosition);
			//Debug.DrawLine(Player.transform.position, Player.transform.position + Quaternion.AngleAxis(-AIBehaviours.AngleOfDeviationToChooseNewPosition, Vector3.up) * rotatedDirectionFromPlayerRear * 15f, Color.blue, minimumTimeToRecalculatePosition);
			//Debug.DrawLine(Player.transform.position, Player.transform.position + Quaternion.AngleAxis(AIBehaviours.AngleOfDeviationToChooseNewPosition, Vector3.up) * rotatedDirectionFromPlayerRear * 15f, Color.blue, minimumTimeToRecalculatePosition);

			var randomDistance = UnityEngine.Random.Range(AIBehaviours.MinimumDistanceToPlayerInCombat, AIBehaviours.MaximumDistanceToPlayerInCombat);
			var positionBehindPlayerWithRandomDistanceAndAngle = Player.transform.position + rotatedDirectionFromPlayerRear * randomDistance;
			destination = positionBehindPlayerWithRandomDistanceAndAngle;
		}
		else
		{
			var directionFromPlayerToSelf = Player.transform.position.DirectionTo(transform.position);
			var randomAngle = UnityEngine.Random.Range(-AIBehaviours.AngleOfDeviationToChooseNewPosition, AIBehaviours.AngleOfDeviationToChooseNewPosition);
			//Debug.DrawLine(transform.position, Player.transform.position, Color.white, 3f);
			//Debug.DrawRay(Player.transform.position + Player.transform.position.DirectionTo(transform.position) * AIBehaviours.MinimumDistanceToPlayerInCombat, Vector3.up * 5f, Color.green, 3f);
			//Debug.DrawRay(Player.transform.position + Player.transform.position.DirectionTo(transform.position) * AIBehaviours.MaximumDistanceToPlayerInCombat, Vector3.up * 5f, Color.green, 3f);
			//Debug.DrawLine(transform.position, transform.position + Quaternion.AngleAxis(-AIBehaviours.AngleOfDeviationToChooseNewPosition, Vector3.up) * -directionFromPlayerToSelf * 15f, Color.blue, 3f);
			//Debug.DrawLine(transform.position, transform.position + Quaternion.AngleAxis(AIBehaviours.AngleOfDeviationToChooseNewPosition, Vector3.up) * -directionFromPlayerToSelf * 15f, Color.blue, 3f);
			var rotatedDirectionFromPlayerToSelf = Quaternion.AngleAxis(randomAngle, Vector3.up) * directionFromPlayerToSelf;
			//Debug.DrawLine(transform.position, transform.position + rotatedDirectionFromPlayerToSelf, Color.magenta, 3f);

			var randomDistance = UnityEngine.Random.Range(AIBehaviours.MinimumDistanceToPlayerInCombat, AIBehaviours.MaximumDistanceToPlayerInCombat);
			var positionFromPlayerToSelfWithRandomDistanceAndAngle = Player.transform.position + rotatedDirectionFromPlayerToSelf * randomDistance;
			destination = positionFromPlayerToSelfWithRandomDistanceAndAngle;
			//Debug.DrawLine(transform.position, positionFromPlayerToSelfWithRandomDistanceAndAngle, Color.green, 3f);
			//Debug.DrawRay(positionFromPlayerToSelfWithRandomDistanceAndAngle, Vector3.up * 5f, Color.green, 3f);
		}

		var pathIsFound = false;
		navPathToDestination = new NavMeshPath();

		if (AIBehaviours.FlankPlayer)
		{
			var cornersToDestination = Player.GetPositionsToFlankPosition(transform.position, destination, AIBehaviours.MinimumDistanceToPlayerInCombat, ref navPathToDestination);
			pathIsFound = cornersToDestination.Length > 0;

			if (DebugNavPath)
			{
				navPathDebugger.positionCount = cornersToDestination.Length;
				navPathDebugger.SetPositions(cornersToDestination);
				navPathDebugger.enabled = true;
			}

			if (pathIsFound)
			{
				this.targetMovementDestination = cornersToDestination[cornersToDestination.Length - 1];
				this.pathToDestinationCorners = cornersToDestination;
				this.pathToDestinationCornerIndex = 0;
				this.timeSinceLastPathSet = 0;
				this.targetMovementNextPosition = null;
			}
		}
		else
		{
			pathIsFound = NavMesh.CalculatePath(transform.position, destination, NavMesh.AllAreas, navPathToDestination);

			if (DebugNavPath)
			{
				navPathDebugger.positionCount = navPathToDestination.corners.Length;
				navPathDebugger.SetPositions(navPathToDestination.corners);
				navPathDebugger.enabled = true;
			}

			if (pathIsFound)
			{
				this.targetMovementDestination = navPathToDestination.corners[navPathToDestination.corners.Length - 1];
				this.pathToDestinationCorners = navPathToDestination.corners;
				this.pathToDestinationCornerIndex = 0;
				this.timeSinceLastPathSet = 0;
				this.targetMovementNextPosition = null;
			}
		}
	}

	protected override void FixedUpdate()
	{
		base.FixedUpdate();

		if (CurrentTurretState == TurretAIState.Dead)
			return;

		RaycastHit hit;
		if (!this.isGrounded && Physics.Raycast(this.GroundCorrectionSensor.position, -this.GroundCorrectionSensor.up, out hit, this.groundCorrectionHoverHeight, Helpers.Masks.Ground))
		{
			Debug.Log($"Trying to push the enemy ship off the ground");
			this.vehicleRigidbody.AddForce(this.GroundCorrectionSensor.up * this.groundCorrectionHoverForce, ForceMode.VelocityChange);
		}
		else if (Physics.Raycast(this.MainThruster.position, this.averageGroundNormal, out hit, this.hoverHeight, Helpers.Masks.Ground))
		{
			this.vehicleRigidbody.AddForce(-this.averageGroundNormal * this.hoverForce * (1f - (hit.distance / this.hoverHeight)), ForceMode.VelocityChange);
		}

		if (Mathf.Abs(this.planarThrustDirection.magnitude) > 0)
		{
			this.vehicleRigidbody.AddForce(this.planarThrustDirection * this.horizontalAcceleration, ForceMode.VelocityChange);
		}

		ApplyRotationalTorqueWithPIDs();
	}

	public void TransitionToState(TurretAIState newState)
	{
		PreviousTurretState = CurrentTurretState;
		OnStateExit(PreviousTurretState, newState);
		CurrentTurretState = newState;
		TurretState = newState;
		TimeEnteredState = Time.time;
		OnStateEnter(newState, PreviousTurretState);
	}

	public void OnStateEnter(TurretAIState state, TurretAIState fromState)
	{
		switch (CurrentTurretState)
		{
			case TurretAIState.Idle:
				timeSinceLastPathSet = AIBehaviours.WaitTimeBeforeMovingToNewPosition;
				break;
			case TurretAIState.Firing:
				//TODO: Start playing Firing animation
				//TODO: Spawn projectile and play sound and particle effect.
				Instantiate(WeaponProjectilePrefab, TurretProjectileSpawnPoint.position, TurretProjectileSpawnPoint.rotation);
				break;
			case TurretAIState.Reloading:
				//TODO: Play animations and do stuff. Then transition back to aiming
				break;
			case TurretAIState.Dead:
				//TODO: Play death animation and sound.
				//Never get out of this.
				break;
		}
	}

	public void OnStateExit(TurretAIState state, TurretAIState toState)
	{
		switch (state)
		{

		}
	}

	public void OnPlayerDetected()
	{
		playerDetected = true;
	}

	public void OnPlayerLostDetection()
	{
		playerDetected = false;
	}

	private void RotateGunTowardForwardPosition()
	{
		TurretGun.rotation = Quaternion.RotateTowards(TurretGun.rotation, transform.rotation, Mathf.Rad2Deg / 2 * TurretRotationSpeed * Time.deltaTime);
	}

	private void RotateGunTowardsTarget()
	{
		targetPredictionPosition = Player.CentreOfMass;
		//TODO: Figure out the target leading on the player based on his rigidbody's movement
		//TODO: Don't forget to have an accuracy factor where it won't 100% lead shots perfectly. That will kill the player fast.
		//Calculate the leading with this turret's projectile speed 
		targetPredictionRotation = Quaternion.LookRotation(targetPredictionPosition - TurretProjectileSpawnPoint.position, Vector3.up);
		TurretGun.rotation = Quaternion.RotateTowards(TurretGun.rotation, targetPredictionRotation, Mathf.Rad2Deg / 2 * TurretRotationSpeed * Time.deltaTime);
	}

	private bool IsGunOnTarget()
	{
		//TODO: Swap out this angle with the "close enough" angle
		var angle = Mathf.Abs(Vector3.Angle(TurretProjectileSpawnPoint.forward, targetPredictionPosition - TurretProjectileSpawnPoint.position));
		//Debug.Log($"Aiming gun on player. Angle: {angle}");
		return angle <= nearAngleEquality;
	}

	public override void ReceiveDamage(int damagePoints)
	{
		base.ReceiveDamage(damagePoints);
	}

	public override void Die()
	{
		TransitionToState(TurretAIState.Dead);
		isDead = true;
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

	//TODO: Update the PIDs for AI to be slower than the Player's.
	private void ApplyRotationalTorqueWithPIDs()
	{
		//Angular velocity
		var angularVelocityError = vehicleRigidbody.angularVelocity * -1;
		Debug.DrawRay(transform.position, vehicleRigidbody.angularVelocity * 10, Color.black);
		var angularVelocityCorrection = angularVelocityPID.UpdatePID(angularVelocityError, Time.fixedDeltaTime);
		Debug.DrawRay(transform.position, angularVelocityCorrection, Color.green);
		vehicleRigidbody.AddTorque(angularVelocityCorrection, ForceMode.Impulse);

		//Heading direction velocity
		var heading = CurrentTurretState == TurretAIState.Dead ? transform.forward :
			AIBehaviours.AlwaysLooksAtPlayerWhenAlert && CurrentTurretState != TurretAIState.Idle ? transform.position.DirectionTo(Player.transform.position) :
			HasNextPosition ? transform.position.DirectionTo(targetMovementNextPosition.Value) :
			transform.forward;

		var desiredHeading = Vector3.ProjectOnPlane(heading, this.isGrounded ? -averageGroundNormal : Vector3.up).normalized;
		Debug.DrawRay(transform.position, desiredHeading, Color.magenta);
		var currentHeading = transform.forward;
		Debug.DrawRay(transform.position, currentHeading * 15, Color.blue);
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

	private void OnValidate()
	{
		StartPosition = transform.position;
		StartDirection = transform.forward;
	}

	private void OnDrawGizmos()
	{
		var lineLength = 1f;

		RaycastHit hit;

		Gizmos.color = Color.cyan;
		Gizmos.DrawRay(transform.position, transform.forward * lineLength);
		Gizmos.color = Color.magenta;
		Gizmos.DrawRay(transform.position, aiInputMove * lineLength);

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
						   this.GroundCorrectionSensor.position + this.GroundCorrectionSensor.transform.up * this.hoverHeight);
		}

		//AI Detection Stuff
		if (playerDetected)
		{
			Debug.DrawRay(TurretGun.position, targetPredictionRotation * Vector3.forward * lineLength, Color.green, 0f, true);

			if (IsGunOnTarget())
				Gizmos.color = Color.red;
			else
				Gizmos.color = Color.yellow;

			Gizmos.DrawRay(TurretProjectileSpawnPoint.position, TurretGun.forward * VehicleRangeSensor.SensorRange);
			Gizmos.DrawSphere(TurretProjectileSpawnPoint.position + TurretGun.forward * VehicleRangeSensor.SensorRange, 2f);

			Gizmos.color = Color.green;
			Gizmos.DrawRay(TurretProjectileSpawnPoint.position, targetPredictionPosition - TurretProjectileSpawnPoint.position);
		}

		if (HasDestination)
		{
			Gizmos.color = Color.magenta;
			Gizmos.DrawLine(transform.position, targetMovementDestination.Value);
			Gizmos.DrawSphere(pathToDestinationCorners[pathToDestinationCorners.Length - 1], 2f);

			if (HasNextPosition)
			{
				Gizmos.color = Color.green;
				Gizmos.DrawSphere(targetMovementNextPosition.Value, 2f);
			}
		}
	}

	//TODO: Turn this into ScriptableObjects to drop onto AI enemy types
	[Serializable]
	public class AIBehaviourModule
	{
		public bool StrafesWhenAlert;
		public float StrafeAlternationTime;
		public bool AlwaysLooksAtPlayerWhenAlert;
		public bool FlankPlayer;
		[Range(0, 60)] public float AngleOfDeviationToChooseNewPosition;
		public float MinimumDistanceToPlayerInCombat;
		public float MaximumDistanceToPlayerInCombat;
		public float WaitTimeBeforeMovingToNewPosition;
	}
}
