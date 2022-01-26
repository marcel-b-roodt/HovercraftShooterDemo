using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerCamera : MonoBehaviour
{
	[Header("Framing")]
	public Camera Camera;
	public Vector2 FollowPointFraming = new Vector2(0f, 0f);
	public float FollowingSharpness = 30f;

	[Header("Distance")]
	public float DefaultDistance = 6f;
	public float MinDistance = 2f;
	public float MaxDistance = 10f;
	public float DistanceMovementSpeed = 10f;
	public float DistanceMovementSharpness = 10f;

	[Header("Rotation")]
	public bool InvertX = false;
	public bool InvertY = false;
	[Range(-90f, 90f)]
	public float DefaultVerticalAngle = 20f;
	[Range(-90f, 90f)]
	public float MinVerticalAngle = -80f;
	[Range(-90f, 90f)]
	public float MaxVerticalAngle = 80f;
	public float RotationSpeed = 10f;
	public float RotationSharpness = 30f;

	[Header("Obstruction")]
	public float ObstructionCheckRadius = 0.5f;
	public LayerMask ObstructionLayers = -1;
	public float ObstructionSharpness = 10000f;

	[Header("Lock-On Targeting")]
	public AIController LockOnTarget;
	public List<AIController> TargetsOnScreen = new List<AIController>(); //Dictionary<int, AIController>();
	public List<AIController> VisibleTargetsOnScreen = new List<AIController>();
	public List<AIController> TargetsInSight = new List<AIController>();
	[ReadOnly] public float WeaponTargetingFOV;
	[ReadOnly] public float WeaponTargetingRange;

	public Transform Transform { get; private set; }
	public Vector3 PlanarDirection { get; private set; }
	public VehicleController FollowCharacter { get; set; }
	public float TargetDistance { get; set; }

	private List<Collider> _internalIgnoredColliders = new List<Collider>();
	private bool _distanceIsObstructed;
	private float _currentDistance;
	private float _targetVerticalAngle;
	private RaycastHit _obstructionHit;
	private int _obstructionCount;
	private RaycastHit[] _obstructions = new RaycastHit[MaxObstructions];
	private float _obstructionTime;
	private Vector3 _currentFollowPosition;

	private const int MaxObstructions = 32;

	void OnValidate()
	{
		DefaultDistance = Mathf.Clamp(DefaultDistance, MinDistance, MaxDistance);
		DefaultVerticalAngle = Mathf.Clamp(DefaultVerticalAngle, MinVerticalAngle, MaxVerticalAngle);
	}

	void Awake()
	{
		Transform = this.transform;

		_currentDistance = DefaultDistance;
		TargetDistance = _currentDistance;

		_targetVerticalAngle = 0f;

		PlanarDirection = Vector3.forward;
	}

	private void Start()
	{
		SetFollowCharacter(GameObject.FindGameObjectWithTag(Helpers.Tags.Player).GetComponent<VehicleController>());
		StartCoroutine(UpdateTargets());
	}

	private void Update()
	{

	}

	IEnumerator UpdateTargets()
	{
		while (enabled)
		{
			GetVisibleTargetsOnScreen();
			GetTargetsInSight();
			GetClosestTargetToCrosshair();
			yield return new WaitForSeconds(0.2f);
		}
	}

	public void AddPotentialTarget(AIController target)
	{
		if (!TargetsOnScreen.Contains(target))
		{
			TargetsOnScreen.Add(target);
		}
	}

	public void RemovePotentialTarget(AIController target)
	{
		if (TargetsOnScreen.Contains(target))
		{
			TargetsOnScreen.Remove(target);

			if (VisibleTargetsOnScreen.Contains(target))
				VisibleTargetsOnScreen.Remove(target);

			if (TargetsInSight.Contains(target))
				TargetsInSight.Remove(target);

			if (LockOnTarget != null && LockOnTarget.GetInstanceID() == target.GetInstanceID())
				LockOnTarget = null;
		}
	}

	public void ResetTargetsInSight(float weaponTargetingFOV, float weaponTargetingRange)
	{
		WeaponTargetingFOV = weaponTargetingFOV;
		WeaponTargetingRange = weaponTargetingRange;
		LockOnTarget = null;
		VisibleTargetsOnScreen.Clear();
		TargetsInSight.Clear();
		GetVisibleTargetsOnScreen();
		GetTargetsInSight();
		GetClosestTargetToCrosshair();
	}

	public void GetVisibleTargetsOnScreen()
	{
		foreach (var target in TargetsOnScreen)
		{
			var visibleEnemy = Vector3.Angle(transform.forward, target.CentreOfMass - transform.position) <= Camera.fieldOfView;
			if (visibleEnemy) //If we are within the camera's FOV angle, do the extra checks
			{
				visibleEnemy = Physics.Raycast(transform.position, target.CentreOfMass - transform.position, out RaycastHit hit, WeaponTargetingRange, Helpers.Masks.ShootableByPlayer)
					&& hit.collider.gameObject.layer == Helpers.LayerIDs.EnemyHurtbox;
			}

			if (visibleEnemy && !VisibleTargetsOnScreen.Contains(target))
			{
				VisibleTargetsOnScreen.Add(target);
			}
			else if (!visibleEnemy && VisibleTargetsOnScreen.Contains(target))
			{
				VisibleTargetsOnScreen.Remove(target);

				if (TargetsInSight.Contains(target))
					TargetsInSight.Remove(target);
			}
		}
	}

	public void GetTargetsInSight()
	{
		foreach (var target in VisibleTargetsOnScreen)
		{
			if (!TargetsInSight.Contains(target) && Vector3.Angle(transform.forward, target.CentreOfMass - transform.position) <= WeaponTargetingFOV)
			{
				TargetsInSight.Add(target);
			}
			else if (TargetsInSight.Contains(target) && Vector3.Angle(transform.forward, target.CentreOfMass - transform.position) > WeaponTargetingFOV)
			{
				TargetsInSight.Remove(target);
			}
		}
	}

	//TODO: Add weapon range as a factor when looking for enemies close to the Target
	public void GetClosestTargetToCrosshair()
	{
		float currentTargetScreenDistanceFromCentre = float.MaxValue;
		AIController closestGameObject = null;

		foreach (var target in TargetsInSight)
		{
			var targetScreenDistanceFromCentre = Vector2.Distance(new Vector2(Camera.pixelWidth / 2, Camera.pixelHeight / 2), Camera.WorldToScreenPoint(target.CentreOfMass));

			if (targetScreenDistanceFromCentre < currentTargetScreenDistanceFromCentre)
			{
				currentTargetScreenDistanceFromCentre = targetScreenDistanceFromCentre;
				closestGameObject = target;
			}
		}

		if (closestGameObject != null)
			LockOnTarget = closestGameObject;
		else
			LockOnTarget = null;
	}

	// Set the transform that the camera will orbit around
	public void SetFollowCharacter(VehicleController character)
	{
		FollowCharacter = character;
		PlanarDirection = FollowCharacter.CameraFollowPoint.forward;
		_currentFollowPosition = FollowCharacter.CameraFollowPoint.position;

		// Ignore the character's collider(s) for camera obstruction checks
		_internalIgnoredColliders.Clear();
		_internalIgnoredColliders.AddRange(FollowCharacter.GetComponentsInChildren<Collider>());
	}

	public void UpdateWithInput(float deltaTime, float zoomInput, Vector2 rotationInput)
	{
		//var cameraUp = FollowCharacter.CameraFollowPoint.up;
		var cameraUp = Vector3.up;

		if (FollowCharacter && FollowCharacter.CameraFollowPoint)
		{
			if (InvertX)
			{
				rotationInput.x *= -1f;
			}
			if (InvertY)
			{
				rotationInput.y *= -1f;
			}

			// Process rotation input
			Quaternion rotationFromInput = Quaternion.Euler(cameraUp * (rotationInput.x * RotationSpeed));
			PlanarDirection = rotationFromInput * PlanarDirection;
			PlanarDirection = Vector3.Cross(cameraUp, Vector3.Cross(PlanarDirection, cameraUp));
			_targetVerticalAngle -= (rotationInput.y * RotationSpeed);
			_targetVerticalAngle = Mathf.Clamp(_targetVerticalAngle, MinVerticalAngle, MaxVerticalAngle);

			// Process distance input
			if (_distanceIsObstructed && Mathf.Abs(zoomInput) > 0f)
			{
				TargetDistance = _currentDistance;
			}
			TargetDistance = zoomInput * DistanceMovementSpeed;
			TargetDistance = Mathf.Clamp(TargetDistance, MinDistance, MaxDistance);

			// Find the smoothed follow position
			_currentFollowPosition = Vector3.Lerp(_currentFollowPosition, FollowCharacter.CameraFollowPoint.position, 1f - Mathf.Exp(-FollowingSharpness * deltaTime));

			// Calculate smoothed rotation
			Quaternion planarRot = Quaternion.LookRotation(PlanarDirection, cameraUp);
			Quaternion verticalRot = Quaternion.Euler(_targetVerticalAngle, 0, 0);
			Quaternion targetRotation = Quaternion.Slerp(Transform.rotation, planarRot * verticalRot, 1f - Mathf.Exp(-RotationSharpness * deltaTime));

			// Apply rotation
			Transform.rotation = targetRotation;

			// Handle obstructions
			{
				RaycastHit closestHit = new RaycastHit();
				closestHit.distance = Mathf.Infinity;
				_obstructionCount = Physics.SphereCastNonAlloc(_currentFollowPosition, ObstructionCheckRadius, -Transform.forward, _obstructions, TargetDistance, ObstructionLayers, QueryTriggerInteraction.Ignore);
				for (int i = 0; i < _obstructionCount; i++)
				{
					bool isIgnored = false;
					for (int j = 0; j < _internalIgnoredColliders.Count; j++)
					{
						if (_internalIgnoredColliders[j] == _obstructions[i].collider)
						{
							isIgnored = true;
							break;
						}
					}
					for (int j = 0; j < FollowCharacter.IgnoredColliders.Count; j++)
					{
						if (FollowCharacter.IgnoredColliders[j] == _obstructions[i].collider)
						{
							isIgnored = true;
							break;
						}
					}

					if (!isIgnored && _obstructions[i].distance < closestHit.distance && _obstructions[i].distance > 0)
					{
						closestHit = _obstructions[i];
					}
				}

				// If obstructions detecter
				if (closestHit.distance < Mathf.Infinity)
				{
					_distanceIsObstructed = true;
					_currentDistance = Mathf.Lerp(_currentDistance, closestHit.distance, 1 - Mathf.Exp(-ObstructionSharpness * deltaTime));
				}
				// If no obstruction
				else
				{
					_distanceIsObstructed = false;
					_currentDistance = Mathf.Lerp(_currentDistance, TargetDistance, 1 - Mathf.Exp(-DistanceMovementSharpness * deltaTime));
				}
			}

			// Find the smoothed camera orbit position
			Vector3 targetPosition = _currentFollowPosition - ((targetRotation * Vector3.forward) * _currentDistance);

			// Handle framing
			targetPosition += Transform.right * FollowPointFraming.x;
			targetPosition += Transform.up * FollowPointFraming.y;

			// Apply position
			Transform.position = targetPosition;
		}
	}

	public Ray GetRayToCurrentTarget(RectTransform crosshair)
	{
		Ray ray;

		if (LockOnTarget != null)
		{
			var targetCentreOfMass = LockOnTarget.CentreOfMass;
			var screenPointForCentreOfMass = this.Camera.WorldToScreenPoint(targetCentreOfMass);
			ray = this.Camera.ScreenPointToRay(screenPointForCentreOfMass);
		}
		else
		{
			//TODO: Make this follow the crosshair on the screen.
			//TODO: Make the PlayerHUD crosshair animate to track the enemy targets, then reset to the centre when not targeting enemy
			ray = this.Camera.ScreenPointToRay(new Vector2(crosshair.position.x, crosshair.position.y));
		}

		return ray;
	}

	private void OnDrawGizmos()
	{
		Gizmos.color = Color.cyan;
		foreach (var target in TargetsInSight)
		{
			Gizmos.DrawLine(transform.position, target.CentreOfMass);
		}

		foreach (var target in TargetsOnScreen)
		{
			var visibleEnemy = Vector3.Angle(transform.forward, target.CentreOfMass - transform.position) <= Camera.fieldOfView;
			if (visibleEnemy) //If we are within the camera's FOV angle, do the extra checks
			{
				visibleEnemy = Physics.Raycast(transform.position, target.CentreOfMass - transform.position, out RaycastHit hit, WeaponTargetingRange, Helpers.Masks.ShootableByPlayer)
					&& hit.transform.gameObject.layer == Helpers.LayerIDs.EnemyHurtbox;
			}

			if (visibleEnemy)
			{
				Gizmos.color = Color.green;
				Gizmos.DrawLine(transform.position, target.CentreOfMass);
			}
			else if (!visibleEnemy)
			{
				Gizmos.color = Color.red;
				Gizmos.DrawLine(transform.position, target.CentreOfMass);
			}
		}

		if (LockOnTarget != null)
		{
			Gizmos.color = Color.green;
			Gizmos.DrawLine(transform.position, LockOnTarget.CentreOfMass);
		}
	}
}