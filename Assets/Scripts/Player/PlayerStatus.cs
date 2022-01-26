using UnityEngine;

public class PlayerStatus : MonoBehaviour
{
	public float MaxHealth;
	[ReadOnly] public float Health;

	public GameObject PlayerHUD;

	public HealthState healthState { get; private set; }

	public string stateName { get { return healthState.ToString(); } }

	void Awake()
	{
		healthState = HealthState.FreeMoving;
		Health = MaxHealth;
	}

	void Start()
	{
		PlayerHUD = GameObject.FindGameObjectWithTag(Helpers.Tags.PlayerHUD);
	}

	public void ReceiveAttack(float damage)
	{
		TakeDamage(damage);
	}

	#region HealthState
	internal void TakeDamage(float damage)
	{
		Health -= damage;

		if (Health <= 0)
			Die();
	}

	internal virtual void Die()
	{
		healthState = HealthState.Dead;
	}
	#endregion

	#region PlayerState
	public bool IsDead()
	{
		return healthState == HealthState.Dead;
	}

	internal bool IsFreeMoving()
	{
		return healthState == HealthState.FreeMoving;
	}

	public void BecomeFreeMoving()
	{
		healthState = HealthState.FreeMoving;
	}
	#endregion
}
