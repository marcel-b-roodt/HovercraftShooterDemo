using UnityEngine;

namespace Helpers
{
	public static class LayerIDs
	{
		public static int EnemyHurtbox = LayerMask.NameToLayer(Helpers.Layers.EnemyHurtbox);
		public static int PlayerHurtbox = LayerMask.NameToLayer(Helpers.Layers.PlayerHurtbox);
	}

	public static class Masks
	{
		public static int EnemyHurtbox = LayerMask.GetMask(Helpers.Layers.EnemyHurtbox);
		public static int Ground = LayerMask.GetMask(Helpers.Layers.Ground);
		public static int ShootableByPlayer = LayerMask.GetMask(Helpers.Layers.EnemyHurtbox, Helpers.Layers.Ground, Helpers.Layers.Environment);
		public static int ShootableByEnemy = LayerMask.GetMask(Helpers.Layers.PlayerHurtbox, Helpers.Layers.Ground, Helpers.Layers.Environment);
	}
}
