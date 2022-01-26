using UnityEngine;

namespace Helpers
{
	[CreateAssetMenuAttribute(menuName = "Firewalker/Math/PID Controller")]
	public class VectorPID : ScriptableObject
	{
		public float pFactor, iFactor, dFactor;

		public VectorPIDInstance Instantiate()
		{
			return new VectorPIDInstance(pFactor, iFactor, dFactor);
		}
	}

	public class VectorPIDInstance
	{
		public float pFactor, iFactor, dFactor;

		private Vector3 integral;
		private Vector3 lastError;

		public VectorPIDInstance(float pFactor, float iFactor, float dFactor)
		{
			this.pFactor = pFactor;
			this.iFactor = iFactor;
			this.dFactor = dFactor;
		}

		public Vector3 UpdatePID(Vector3 currentError, float timeFrame)
		{
			integral += currentError * timeFrame;
			var deriv = (currentError - lastError) / timeFrame;
			lastError = currentError;
			return currentError * pFactor
				+ integral * iFactor
				+ deriv * dFactor;
		}
	}
}
