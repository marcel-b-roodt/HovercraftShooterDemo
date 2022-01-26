using UnityEngine;

[CreateAssetMenuAttribute(menuName = "Firewalker/Player/Weapons/Railgun")]
public class RailgunWeapon : EquippableWeapon
{
	public override string Name { get { return "Railgun"; } }
	public override VehicleController.WeaponType Type { get { return VehicleController.WeaponType.Railgun; } }
	public float MaxChargeReleaseFireTime = 1.5f;
	public float MaxChargeUpTime = 4f;
	public float MinimumChargeUpTime = 0.5f;
	public float MaxChargeHoldTime = 2f;

	public override float WeaponFireRate => MaxChargeReleaseFireTime * (CurrentChargeUpTime / MaxChargeUpTime); //TODO: Convert the charge time percentage into a Fire Rate variable. We shouldn't be able to spam the railgun
	public override string WeaponText => $"{ Mathf.RoundToInt(CurrentChargeUpTime / MaxChargeUpTime * 100) }";
	public bool FullyCharged => (CurrentChargeUpTime == MaxChargeUpTime);

	[ReadOnly] public float CurrentChargeUpTime;
	[ReadOnly] public float CurrentHoldTime;
	[ReadOnly] public float ExpectedFireRate;

	public ParticleSystem[] BulletTrailSystems;

	public override void WeaponUpdate(bool fireHeld)
	{
		base.WeaponUpdate(fireHeld);

		if (CurrentFireTime > 0)
		{
			CurrentChargeUpTime = 0;
			CurrentHoldTime = 0;
		}
	}

	public override void Initialise()
	{
		CurrentChargeUpTime = 0;
		CurrentHoldTime = 0;
		ExpectedFireRate = WeaponFireRate;
	}

	protected override bool CanFire(bool fireHeld)
	{
		if (fireHeld)
		{
			if (CurrentChargeUpTime < MaxChargeUpTime)
				CurrentChargeUpTime = Mathf.Clamp(CurrentChargeUpTime + Time.deltaTime, 0, MaxChargeUpTime);
			else if (CurrentChargeUpTime >= MaxChargeUpTime)
				CurrentHoldTime = Mathf.Clamp(CurrentHoldTime + Time.deltaTime, 0, MaxChargeHoldTime);

			if (CurrentHoldTime >= MaxChargeHoldTime)
				return true;
		}
		else
		{
			if (CurrentChargeUpTime >= MinimumChargeUpTime)
				return true;
			else
			{
				CurrentChargeUpTime = 0;
				CurrentHoldTime = 0;
			}
		}

		ExpectedFireRate = WeaponFireRate;
		return false;
	}

	protected override void Fire()
	{
		var currentBulletSpawnIndex = GetBulletSpawnIndex();
		var currentBulletSpawnPoint = this.BulletSpawnPoints[currentBulletSpawnIndex];
		var currentBulletTrailSystem = this.BulletTrailSystems[currentBulletSpawnIndex];

		Ray ray = this.PlayerCamera.GetRayToCurrentTarget(this.Crosshair);
		if (Physics.Raycast(ray, out RaycastHit hit, WeaponRange, Helpers.Masks.ShootableByPlayer))
		{
			if (hit.collider.gameObject.layer == Helpers.LayerIDs.EnemyHurtbox)
			{
				var enemyComponent = hit.collider.gameObject.GetComponent<EnemyHurtbox>().Parent;
				var weaponDamage = (int)(WeaponDamage * (CurrentChargeUpTime / MaxChargeUpTime));
				enemyComponent.ReceiveDamage(weaponDamage);
				Debug.Log("Hit enemy");
			}
			else
				Debug.Log("Hit environment");

			currentBulletSpawnPoint.LookAt(hit.point, Vector3.up);
		}
		else
		{
			Debug.Log("Missed shot");
			currentBulletSpawnPoint.LookAt(ray.GetPoint(WeaponRange), Vector3.up);
		}

		currentBulletTrailSystem.Stop();
		currentBulletTrailSystem.Play();
	}

	protected override void AltFire()
	{

	}
}
