using UnityEngine;

namespace Helpers
{
	public static class VectorExtensions
	{
		public static Vector3 XZVector(this Vector3 vector)
		{
			return new Vector3(vector.x, 0, vector.z);
		}

		public static Vector3 DirectionTo(this Vector3 vector, Vector3 otherVector)
		{
			return (otherVector - vector).normalized;
		}
	}
}
