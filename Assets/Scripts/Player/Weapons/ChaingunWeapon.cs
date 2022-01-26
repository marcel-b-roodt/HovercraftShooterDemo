using UnityEngine;

[CreateAssetMenuAttribute(menuName = "Firewalker/Player/Weapons/Chaingun")]
public class ChaingunWeapon : EquippableWeapon
{
	public override string Name { get { return "Chaingun"; } }
	public override VehicleController.WeaponType Type { get { return VehicleController.WeaponType.Chaingun; } }
	public float SpinUpTime = 8f;
	public float WeaponMaxHeat = 100f;
	[Range(0, 1)] public float WeaponHeatResetPercentage = 0.3f;
	public float WeaponHeatBuildUpRate = 4.2f;
	public float WeaponHeatDecayRate = 28f;
	public float WeaponMinFireRate = 30;
	public float WeaponMaxFireRate = 1800;
	public float WeaponFireRateDecayMultiplier = 4f;

	public override float WeaponFireRate => ConvertFromRPM(CurrentWeaponFireRateRPM);
	public override string WeaponText => $"{ Mathf.RoundToInt(CurrentWeaponHeat / WeaponMaxHeat * 100) }";

	[ReadOnly] public float CurrentSpinUpTime;
	[ReadOnly] public float CurrentWeaponFireRateRPM;
	[ReadOnly] public float CurrentWeaponHeat;
	[ReadOnly] public bool WeaponMaxHeatReached;

	public ParticleSystem[] BulletTrailSystems;

	public override void WeaponUpdate(bool fireHeld)
	{
		base.WeaponUpdate(fireHeld);

		this.CurrentWeaponHeat = Mathf.Clamp(this.CurrentWeaponHeat - (WeaponHeatDecayRate * Time.deltaTime), 0, WeaponMaxHeat);
		if (CurrentWeaponHeat <= WeaponMaxHeat * WeaponHeatResetPercentage)
		{
			WeaponMaxHeatReached = false;
		}

		if (fireHeld)
			this.CurrentSpinUpTime = Mathf.Clamp(this.CurrentSpinUpTime + Time.deltaTime, 0, SpinUpTime);
		else
			this.CurrentSpinUpTime = Mathf.Clamp(this.CurrentSpinUpTime - (WeaponFireRateDecayMultiplier * Time.deltaTime), 0, SpinUpTime);

		this.CurrentWeaponFireRateRPM = Mathf.Lerp(WeaponMinFireRate, WeaponMaxFireRate, this.CurrentSpinUpTime / SpinUpTime);
	}

	public override void Initialise()
	{
		CurrentSpinUpTime = 0;
		CurrentWeaponFireRateRPM = 0;
		CurrentWeaponHeat = 0;
		WeaponMaxHeatReached = false;
	}

	protected override bool CanFire(bool fireHeld)
	{
		return fireHeld && !WeaponMaxHeatReached;
	}

	protected override void Fire()
	{
		this.CurrentWeaponHeat = Mathf.Clamp(this.CurrentWeaponHeat + WeaponHeatBuildUpRate, 0, WeaponMaxHeat);
		if (CurrentWeaponHeat == WeaponMaxHeat)
		{
			WeaponMaxHeatReached = true;
		}

		var currentBulletSpawnIndex = GetBulletSpawnIndex();
		var currentBulletSpawnPoint = this.BulletSpawnPoints[currentBulletSpawnIndex];
		var currentBulletTrailSystem = this.BulletTrailSystems[currentBulletSpawnIndex];

		Ray ray = this.PlayerCamera.GetRayToCurrentTarget(this.Crosshair);
		if (Physics.Raycast(ray, out RaycastHit hit, WeaponRange, Helpers.Masks.ShootableByPlayer))
		{
			if (hit.collider.gameObject.layer == Helpers.LayerIDs.EnemyHurtbox)
			{
				var enemyComponent = hit.collider.gameObject.GetComponent<EnemyHurtbox>().Parent;
				enemyComponent.ReceiveDamage(WeaponDamage);
				//Debug.Log("Hit enemy");
			}
			//else
			//Debug.Log("Hit environment");

			currentBulletSpawnPoint.LookAt(hit.point, Vector3.up);
		}
		else
		{
			//Debug.Log("Missed shot");
			currentBulletSpawnPoint.LookAt(ray.GetPoint(WeaponRange), Vector3.up);
		}
		currentBulletTrailSystem.Stop();
		currentBulletTrailSystem.Play();
	}

	protected override void AltFire()
	{

	}
}
