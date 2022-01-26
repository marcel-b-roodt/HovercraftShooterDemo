using UnityEngine;

public class EnemyHurtbox : MonoBehaviour
{
	public AIController Parent;

	public void Start()
	{
		if (Parent == null)
			Debug.LogError($"Hurtbox {name}-{GetInstanceID()} lacks a parent.");
	}
}
