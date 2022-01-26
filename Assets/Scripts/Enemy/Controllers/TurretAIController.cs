using SensorToolkit;
using UnityEngine;

public class TurretAIController : AIController
{
	public enum TurretAIState
	{
		Hidden,
		Raising,
		Aiming,
		Firing,
		Reloading,
		Lowering,
		Dead,
	}

	[Header("Components")]
	public RangeSensor TurretRangeSensor;
	public Transform Turret;
	public Transform TurretGun;
	public Transform TurretProjectileSpawnPoint;
	public Transform TurretLoweredPoint;
	public Transform TurretRaisedPoint;

	[Header("Enemy Properties")]
	[ReadOnly] public Vector3 StartPosition;
	[ReadOnly] public Vector3 StartDirection;
	[ReadOnly] public Quaternion StartRotation;
	[ReadOnly] public TurretAIState TurretState;

	[Header("General Properties")]
	public float TurretRaiseLowerSpeed = 2.5f;
	public float TurretRotationSpeed = 3.5f;
	[Range(0, 1)] public float TrackingPrediction = 0f; //TODO: Implement. 0 - Poor tracking. Shoots directly at the player. 1 - Perfect tracking. Shoots exactly at the position the player would be based on the projectile speed.
	public Transform WeaponProjectilePrefab;
	public float WeaponProjectileSpeed = 5f;
	public float WeaponFireRate = 3.5f;

	public TurretAIState CurrentTurretState { get; private set; }
	public TurretAIState PreviousTurretState { get; private set; }
	public float TimeEnteredState { get; private set; }
	public float TimeSinceEnteringState { get { return Time.time - TimeEnteredState; } }

	private bool playerDetected;
	private Vector3 targetPredictionPosition;
	private Quaternion targetPredictionRotation;

	protected override void Start()
	{
		base.Start();

		StartPosition = transform.position;
		StartDirection = transform.forward;
		StartRotation = transform.rotation;
		TransitionToState(TurretAIState.Hidden);
		Turret.position = TurretLoweredPoint.position;
	}

	protected override void Update()
	{
		base.Update();

		switch (CurrentTurretState)
		{
			case TurretAIState.Hidden:
				if (playerDetected)
				{
					TransitionToState(TurretAIState.Raising);
				}
				break;

			case TurretAIState.Raising:
				if (!playerDetected)
				{
					TransitionToState(TurretAIState.Lowering);
				}

				if (IsTurretFullyRaised())
				{
					Turret.position = TurretRaisedPoint.position;
					TransitionToState(TurretAIState.Aiming);
				}
				else
				{
					Turret.position = Vector3.Lerp(Turret.position, TurretRaisedPoint.position, TurretRaiseLowerSpeed * Time.deltaTime);
				}
				break;

			case TurretAIState.Aiming:
				if (!playerDetected)
				{
					TransitionToState(TurretAIState.Lowering);
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

			case TurretAIState.Lowering:
				RotateGunTowardStartRotation();

				if (playerDetected)
				{
					TransitionToState(TurretAIState.Raising);
				}

				if (IsTurretFullyLowered())
				{
					Turret.position = TurretLoweredPoint.position;
					Turret.rotation = StartRotation;
					TransitionToState(TurretAIState.Hidden);
				}
				else
				{
					Turret.position = Vector3.Lerp(Turret.position, TurretLoweredPoint.position, TurretRaiseLowerSpeed * Time.deltaTime);
				}
				break;
		}
	}

	//TODO: Make this generic across enemies. This should be called for all of them.
	//TODO: Consider making this trigger once every few frames so that it's not so taxing

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

	private void RotateGunTowardStartRotation()
	{
		TurretGun.rotation = Quaternion.RotateTowards(TurretGun.rotation, StartRotation, Mathf.Rad2Deg / 2 * TurretRotationSpeed * Time.deltaTime);
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

	private bool IsTurretFullyRaised()
	{
		return Vector3.Distance(Turret.position, TurretRaisedPoint.position) <= nearVectorEquality;
	}

	private bool IsTurretFullyLowered()
	{
		return Vector3.Distance(Turret.position, TurretLoweredPoint.position) <= nearVectorEquality;
	}

	public override void ReceiveDamage(int damagePoints)
	{
		base.ReceiveDamage(damagePoints);
	}

	public override void Die()
	{
		TransitionToState(TurretAIState.Dead);
	}

	private void OnValidate()
	{
		StartPosition = transform.position;
		StartDirection = transform.forward;
	}

	private void OnDrawGizmos()
	{
		if (playerDetected)
		{
			var lineLength = 2f;
			Debug.DrawRay(TurretGun.position, targetPredictionRotation * Vector3.forward * lineLength, Color.green, 0f, true);

			if (IsGunOnTarget())
				Gizmos.color = Color.red;
			else
				Gizmos.color = Color.yellow;

			Gizmos.DrawRay(TurretProjectileSpawnPoint.position, TurretGun.forward * TurretRangeSensor.SensorRange);
			Gizmos.DrawSphere(TurretProjectileSpawnPoint.position + TurretGun.forward * TurretRangeSensor.SensorRange, 2f);

			Gizmos.color = Color.green;
			Gizmos.DrawRay(TurretProjectileSpawnPoint.position, targetPredictionPosition - TurretProjectileSpawnPoint.position);
		}
	}
}
