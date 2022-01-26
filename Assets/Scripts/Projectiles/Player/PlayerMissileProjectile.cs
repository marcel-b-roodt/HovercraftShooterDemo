using UnityEngine;

//TODO: Create Base Missile Projectile
public class PlayerMissileProjectile : MonoBehaviour
{
	private const float MinimumDamageFromProjectile = 0.2f; //TODO: Make this standard

	[Header("Components")]
	public Rigidbody ProjectileRigidbody;
	[ReadOnly] public AIController Enemy;
	[ReadOnly] public Vector3 TargetPosition;

	[Header("Properties")]
	public float ProjectileSpeed;
	public int ProjectileDamage;
	public float ProjectileAOERadius;
	public float ProjectileMaxAliveTime;
	public float ProjectileTrackingAngle;
	public float ProjectileTrackingHoldTime;
	public float ProjectileTrackingSpeed;

	//private int enemyLayer;
	//private int shootableLayerMask;

	private float aliveTime;
	private float lostTrackingTimer = 0f;
	private bool lostTracking = false;
	private Quaternion targetTrackingRotation;

	private Collider Hitbox;
	private Vector3 CentreOfMass { get { return Hitbox.bounds.center; } }

	private void Awake()
	{
		Hitbox = GetComponent<Collider>();
	}

	private void Start()
	{
		//enemyLayer = 1 << LayerMask.GetMask(Helpers.Layers.EnemyHurtbox);
		//shootableLayerMask = LayerMask.GetMask(Helpers.Layers.EnemyHurtbox, Helpers.Layers.Ground, Helpers.Layers.Environment);
		aliveTime = Time.time;
	}

	public void InstantiateMissile(AIController target, Vector3 targetPosition)
	{
		Enemy = target; //Taken from the PlayerCamera component. We either have a target enemy to follow, or we are heading to a position that we aimed at
		TargetPosition = targetPosition;
	}

	private void Update()
	{
		if (Enemy != null)
		{
			//General Target Tracking Logic
			if (!lostTracking)
			{
				if (IsTrackingEnemy())
				{
					lostTrackingTimer = 0;
				}
				else
				{
					lostTrackingTimer += Time.deltaTime;
				}

				targetTrackingRotation = Quaternion.LookRotation(Enemy.CentreOfMass - transform.position, Vector3.up);
				transform.rotation = Quaternion.RotateTowards(transform.rotation, targetTrackingRotation, Mathf.Rad2Deg / 2 * ProjectileTrackingSpeed * Time.deltaTime);

				if (lostTrackingTimer >= ProjectileTrackingHoldTime)
				{
					lostTracking = true;
				}
			}
		}
		else
		{
			targetTrackingRotation = Quaternion.LookRotation(TargetPosition - transform.position, Vector3.up);
			transform.rotation = Quaternion.RotateTowards(transform.rotation, targetTrackingRotation, Mathf.Rad2Deg / 2 * ProjectileTrackingSpeed * Time.deltaTime);
		}

		if (Time.time - aliveTime >= ProjectileMaxAliveTime)
		{
			//Debug.Log($"Projectile reached max lifespan");
			Explode();
		}
	}

	private void FixedUpdate()
	{
		ProjectileRigidbody.velocity = transform.forward * ProjectileSpeed;
	}

	private void OnCollisionEnter(Collision collision)
	{
		//Debug.Log($"Projectile collided with {collision.gameObject.name}");
		Explode();
	}

	private bool IsTrackingEnemy()
	{
		if (Enemy == null)
			return false;

		var trackingAngle = Mathf.Abs(Vector3.Angle(transform.forward, Enemy.CentreOfMass - transform.position));
		return trackingAngle <= ProjectileTrackingAngle;
	}

	private void Explode()
	{
		var shootableColliders = Physics.OverlapSphere(CentreOfMass, ProjectileAOERadius, Helpers.Masks.ShootableByPlayer);

		foreach (var collider in shootableColliders)
		{
			if (collider.gameObject.layer == Helpers.LayerIDs.EnemyHurtbox
				&& Physics.Raycast(CentreOfMass, (collider.ClosestPoint(CentreOfMass) - CentreOfMass) * ProjectileAOERadius, out RaycastHit hit, ProjectileAOERadius, Helpers.Masks.ShootableByPlayer)
				&& hit.collider.gameObject.layer == Helpers.LayerIDs.EnemyHurtbox)
			{
				//var gotHit = Physics.Raycast(CentreOfMass, (collider.ClosestPoint(CentreOfMass) - CentreOfMass) * ProjectileAOERadius, out RaycastHit hit, ProjectileAOERadius, Helpers.Masks.ShootableByPlayer);
				//Debug.DrawRay(CentreOfMass, (collider.ClosestPoint(CentreOfMass) - CentreOfMass) * ProjectileAOERadius, Color.white, 30f);

				var distanceFromExplosion = Vector3.Distance(CentreOfMass, hit.point);
				var enemyComponent = hit.collider.gameObject.GetComponent<EnemyHurtbox>().Parent;
				DealDamageToEnemy(enemyComponent, distanceFromExplosion);
			}
		}
		//TODO: Explode sound
		//TODO: Explode particle effect
		Destroy(gameObject);
	}

	private void DealDamageToEnemy(AIController enemy, float distanceFromExplosion)
	{
		var percentageDistanceFromExplosion = Mathf.Clamp(1 - (distanceFromExplosion / ProjectileAOERadius), MinimumDamageFromProjectile, 1);
		var damage = (int)(percentageDistanceFromExplosion * ProjectileDamage);

		enemy.ReceiveDamage(damage);
	}

	private void OnDrawGizmos()
	{
		float lineLength = 10f;

		//Gizmos.color = Color.cyan;
		//Gizmos.DrawWireSphere(transform.position, ProjectileAOERadius);

		Gizmos.color = Color.green;
		Gizmos.DrawRay(transform.position, transform.forward * lineLength);

		if (Enemy != null)
		{
			if (IsTrackingEnemy())
				Gizmos.color = Color.red;
			else
				Gizmos.color = Color.yellow;

			if (!lostTracking)
				Gizmos.DrawRay(transform.position, Enemy.CentreOfMass - transform.position);
			else
			{
				Gizmos.color = Color.white;
				Gizmos.DrawSphere(transform.position, 1f);
			}
		}
		else
		{
			Gizmos.color = Color.cyan;
			Gizmos.DrawRay(transform.position, TargetPosition - transform.position);
		}
	}
}
