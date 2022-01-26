using UnityEngine;

[RequireComponent(typeof(SphereCollider))]
[RequireComponent(typeof(Rigidbody))]
public class EnemyPlasmaProjectile : MonoBehaviour
{
	private const float MinimumDamageFromProjectile = 0.2f; //TODO: Make this standard

	//TODO: Refactor Projectiles into Generic Projectile Class
	[Header("Components")]
	public Rigidbody ProjectileRigidbody;

	[Header("Properties")]
	public float ProjectileSpeed;
	public int ProjectileDamage;
	public float ProjectileAOERadius;
	public float ProjectileMaxAliveTime;

	private float aliveTime;

	private Collider Hitbox;
	private Vector3 CentreOfMass { get { return Hitbox.bounds.center; } }

	private void Awake()
	{
		Hitbox = GetComponent<Collider>();
	}

	private void Start()
	{
		aliveTime = Time.time;
	}

	private void Update()
	{
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
		//Debug.Log($"Player received {damage}dmg from plasma ball.");
		vehicleController.ReceiveDamage(damage);
	}

	private void OnDrawGizmos()
	{
		Gizmos.color = Color.red;
		Gizmos.DrawWireSphere(transform.position, ProjectileAOERadius);
	}
}
