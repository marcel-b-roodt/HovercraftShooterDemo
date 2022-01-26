using UnityEngine;

[CreateAssetMenuAttribute(menuName = "Firewalker/Player/Weapons/SmartMissiles")]
public class SmartMissileWeapon : EquippableWeapon
{
	public override string Name { get { return "Smart Missiles"; } }
	public override VehicleController.WeaponType Type { get { return VehicleController.WeaponType.Missiles; } }
	public int MaxAmmoCapacity;
	[ReadOnly] public int CurrentAmmoCapacity;
	public float AmmoRefreshRate; //Time it takes for a single missile to replenish
	[ReadOnly] public float CurrentAmmoRefreshTime;

	public Transform MissileProjectile;
	public AIController LockOnTarget { get { return PlayerCamera.LockOnTarget; } }

	public override string WeaponText => $"{CurrentAmmoCapacity} / {MaxAmmoCapacity}";

	public override void WeaponUpdate(bool fireHeld)
	{
		base.WeaponUpdate(fireHeld);

		if (CurrentAmmoCapacity < MaxAmmoCapacity)
		{
			if (this.CurrentAmmoRefreshTime > 0)
			{
				this.CurrentAmmoRefreshTime = Mathf.Clamp(this.CurrentAmmoRefreshTime - Time.deltaTime, 0, AmmoRefreshRate);
			}
			else if (this.CurrentAmmoRefreshTime == 0)
			{
				CurrentAmmoCapacity++;
				this.CurrentAmmoRefreshTime = AmmoRefreshRate;
			}
		}
		else if (CurrentAmmoCapacity == MaxAmmoCapacity)
		{
			CurrentAmmoRefreshTime = AmmoRefreshRate;
		}

	}

	public override void Initialise()
	{
		CurrentAmmoCapacity = MaxAmmoCapacity;
		CurrentAmmoRefreshTime = AmmoRefreshRate;
	}

	protected override bool CanFire(bool fireHeld)
	{
		return base.CanFire(fireHeld) && CurrentAmmoCapacity > 0;
	}

	protected override void Fire()
	{
		var currentBulletSpawnIndex = GetBulletSpawnIndex();
		var currentBulletSpawnPoint = BulletSpawnPoints[currentBulletSpawnIndex];

		AIController missileTarget = null;
		Vector3 missileTargetPosition;

		if (PlayerCamera.LockOnTarget != null)
		{
			missileTarget = LockOnTarget;
			missileTargetPosition = LockOnTarget.transform.position;
		}
		else
		{
			Ray ray = this.PlayerCamera.Camera.ScreenPointToRay(new Vector2(Crosshair.position.x, Crosshair.position.y));
			if (Physics.Raycast(ray, out RaycastHit hit, WeaponRange, Helpers.Masks.ShootableByPlayer))
			{
				missileTargetPosition = hit.point;
			}
			else
			{
				missileTargetPosition = ray.direction * WeaponRange;
			}
		}

		var missile = Object.Instantiate(MissileProjectile, currentBulletSpawnPoint.position, currentBulletSpawnPoint.rotation);
		var playerMissileProjectile = missile.GetComponent<PlayerMissileProjectile>();
		playerMissileProjectile.InstantiateMissile(missileTarget, missileTargetPosition);
		CurrentAmmoCapacity--;
	}

	protected override void AltFire()
	{

	}
}
