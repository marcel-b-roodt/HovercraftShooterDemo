using UnityEngine;

public abstract class AIController : MonoBehaviour
{
	protected const float nearVectorEquality = 0.02f;
	protected const float nearAngleEquality = 1f;

	[Header("Components")]
	public Renderer[] Renderers;
	public Collider Hurtbox; //TODO: Possibly make this an array of colliders
							 //TODO: If we replace to multi-Hurtbox, then we should have a collider that only represents the shape of the enemy for the targeting reticle
							 //TODO: If we want to have multiple hurtboxes, then we should make centre of mass the average centre of all hurtboxes.
	[ReadOnly] public VehicleController Player;
	[ReadOnly] public PlayerCamera PlayerCamera;
	[ReadOnly] public Vector3 PlayerLastSeenPosition;

	[Header("Enemy Properties")]
	public string EnemyTypeName;
	public int MaxHealth;
	[ReadOnly] public int CurrentHealth;

	public Vector3 CentreOfMass { get { return Hurtbox.bounds.center; } }

	protected virtual void Awake()
	{
		Renderers = GetComponentsInChildren<Renderer>();
		foreach (var renderer in Renderers)
		{
			if (renderer.isVisible)
			{
				PlayerCamera.AddPotentialTarget(this);
				break;
			}
		}
	}

	protected virtual void Start()
	{
		Player = GameObject.FindGameObjectWithTag(Helpers.Tags.Player).GetComponent<VehicleController>();
		PlayerCamera = GameObject.FindGameObjectWithTag(Helpers.Tags.PlayerCamera).GetComponent<PlayerCamera>();

		Initialise();
	}

	protected virtual void Update()
	{
		CalculatePotentialTargetForPlayer();
	}

	protected virtual void FixedUpdate()
	{

	}

	protected virtual void Initialise()
	{
		CurrentHealth = MaxHealth;
	}


	protected void CalculatePotentialTargetForPlayer()
	{
		bool anyRendererVisible = false;

		foreach (var renderer in Renderers)
		{
			if (renderer.isVisible)
			{
				anyRendererVisible = true;
				PlayerCamera.AddPotentialTarget(this);
				break;
			}
		}

		if (!anyRendererVisible)
		{
			PlayerCamera.RemovePotentialTarget(this);
		}
	}

	public virtual void ReceiveDamage(int damagePoints)
	{
		CurrentHealth = Mathf.Clamp(CurrentHealth - damagePoints, 0, MaxHealth);

		//Debug.Log($"Enemy {gameObject.name} received {damagePoints}dmg. HP is now {CurrentHealth}");

		if (CurrentHealth <= 0)
			Die();
	}

	public abstract void Die();
}
