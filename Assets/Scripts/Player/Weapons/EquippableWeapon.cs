using UnityEngine;

public abstract class EquippableWeapon : ScriptableObject
{
	[Range(0, 20)] public float TargetingAngle;
	public int WeaponDamage;
	public float WeaponSwitchDelay;
	public float WeaponRange;
	public float WeaponFireRateRPM;
	[ReadOnly] public float CurrentFireTime;
	[ReadOnly] public PlayerCamera PlayerCamera;
	[ReadOnly] public RectTransform Crosshair;
	public Transform[] BulletSpawnPoints;

	public abstract string Name { get; }
	public abstract VehicleController.WeaponType Type { get; }
	public virtual float WeaponFireRate { get { return ConvertFromRPM(WeaponFireRateRPM); } } //The effective millisecond delay between rounds
	public virtual string WeaponText { get; }

	private int bulletSpawnPointIndex; //Cycles through spawn points.

	public float ConvertFromRPM(float roundsPerMinute)
	{
		return 1 / (roundsPerMinute / 60);
	}

	public void SwitchWeapon()
	{
		CurrentFireTime = WeaponSwitchDelay;
		PlayerCamera.ResetTargetsInSight(TargetingAngle, WeaponRange);
	}

	public virtual void WeaponUpdate(bool fireHeld)
	{
		CheckFireRate(fireHeld);
	}

	protected virtual bool CanFire(bool fireHeld)
	{
		return fireHeld;
	}

	protected int GetBulletSpawnIndex()
	{
		var index = this.bulletSpawnPointIndex;
		this.bulletSpawnPointIndex++;

		if (this.bulletSpawnPointIndex >= this.BulletSpawnPoints.Length)
			this.bulletSpawnPointIndex = 0;

		return index;
	}

	private void CheckFireRate(bool fireHeld)
	{
		if (this.CurrentFireTime == 0)
		{
			if (CanFire(fireHeld))
			{
				Fire();
				this.CurrentFireTime = WeaponFireRate;
			}
		}

		this.CurrentFireTime = Mathf.Clamp(this.CurrentFireTime - Time.deltaTime, 0, Mathf.Max(WeaponFireRate, WeaponSwitchDelay));
	}

	public abstract void Initialise();
	protected abstract void Fire();
	protected abstract void AltFire();

}
