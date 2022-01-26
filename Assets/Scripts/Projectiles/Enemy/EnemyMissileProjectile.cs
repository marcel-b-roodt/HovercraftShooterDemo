using UnityEngine;

//TODO: Create Base Missile Projectile
public class EnemyMissileProjectile : MonoBehaviour
{
	private const float MinimumDamageFromProjectile = 0.2f; //TODO: Make this standard

	[Header("Components")]
	public Rigidbody ProjectileRigidbody;
	[ReadOnly] public VehicleController Player;

	[Header("Properties")] //TODO: Possibly transfer these over to the enemy's properties. These can be initialised by the Enemy
	public float ProjectileSpeed;
	public int ProjectileDamage;
	public float ProjectileAOERadius;
	public float ProjectileMaxAliveTime;
	public float ProjectileTrackingAngle;
	public float ProjectileTrackingHoldTime;
	public float ProjectileTrackingSpeed;

	//private int playerHurtboxLayerMask;

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
		Player = GameObject.FindGameObjectWithTag(Helpers.Tags.Player).GetComponent<VehicleController>();
		//playerHurtboxLayerMask = Helpers.Masks.Player; //1 << LayerMask.GetMask(Helpers.Layers.PlayerHurtbox);
		aliveTime = Time.time;
	}

	private void Update()
	{
		if (!lostTracking)
		{
			if (IsTrackingPlayer())
			{
				lostTrackingTimer = 0;
			}
			else
			{
				lostTrackingTimer += Time.deltaTime;
			}

			targetTrackingRotation = Quaternion.LookRotation(Player.CentreOfMass - transform.position, Vector3.up);
			transform.rotation = Quaternion.RotateTowards(transform.rotation, targetTrackingRotation, Mathf.Rad2Deg / 2 * ProjectileTrackingSpeed * Time.deltaTime);

			if (lostTrackingTimer >= ProjectileTrackingHoldTime)
			{
				lostTracking = true;
			}
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

	private bool IsTrackingPlayer()
	{
		if (Player == null)
			return false;

		var trackingAngle = Mathf.Abs(Vector3.Angle(transform.forward, Player.CentreOfMass - transform.position));
		return trackingAngle <= ProjectileTrackingAngle;
	}

	private void Explode()
	{
		var shootableColliders = Physics.OverlapSphere(CentreOfMass, ProjectileAOERadius, Helpers.Masks.ShootableByEnemy);

		foreach (var collider in shootableColliders)
		{
			if (collider.gameObject.layer == Helpers.LayerIDs.PlayerHurtbox
				&& Physics.Raycast(CentreOfMass, (collider.ClosestPoint(CentreOfMass) - CentreOfMass) * ProjectileAOERadius, out RaycastHit hit, ProjectileAOERadius, Helpers.Masks.ShootableByEnemy))
			{
				//var gotHit = Physics.Raycast(CentreOfMass, (collider.ClosestPoint(CentreOfMass) - CentreOfMass) * ProjectileAOERadius, out RaycastHit hit, ProjectileAOERadius, Helpers.Masks.ShootableByEnemy);
				//Debug.DrawRay(CentreOfMass, (collider.ClosestPoint(CentreOfMass) - CentreOfMass) * ProjectileAOERadius, Color.white, 30f);
				var distanceFromExplosion = Vector3.Distance(CentreOfMass, hit.point);
				DealDamageToPlayer(distanceFromExplosion);
			}
		}

		//TODO: Explode sound
		//TODO: Explode particle effect
		Destroy(gameObject);
	}

	//TODO: Turn into a generic method
	private void DealDamageToPlayer(float distanceFromExplosion)
	{
		var percentageDistanceFromExplosion = Mathf.Clamp(1 - (distanceFromExplosion / ProjectileAOERadius), MinimumDamageFromProjectile, 1);
		var damage = (int)(percentageDistanceFromExplosion * ProjectileDamage);

		var player = GameObject.FindGameObjectWithTag(Helpers.Tags.Player);
		var vehicleController = player.GetComponent<VehicleController>();
		//Debug.Log($"Player received {damage}dmg from missile.");
		vehicleController.ReceiveDamage(damage);
	}

	private void OnDrawGizmos()
	{
		float lineLength = 10f;

		Gizmos.color = Color.red;
		Gizmos.DrawWireSphere(transform.position, ProjectileAOERadius);

		Gizmos.color = Color.green;
		Gizmos.DrawRay(transform.position, transform.forward * lineLength);

		if (IsTrackingPlayer())
			Gizmos.color = Color.red;
		else
			Gizmos.color = Color.yellow;

		if (!lostTracking)
			Gizmos.DrawRay(transform.position, Player.CentreOfMass - transform.position);
		else
		{
			Gizmos.color = Color.white;
			Gizmos.DrawSphere(transform.position, 1f);
		}
	}
}
