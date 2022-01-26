using System.Linq;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class PlayerHUD : MonoBehaviour
{
	private const float ReticleScaleFactor = 1.5f;

	public Image Crosshair;
	public Sprite[] CrosshairImages;
	public TextMeshProUGUI JumpBoostValueText;
	public TextMeshProUGUI ShipSpeedValueText;
	public TextMeshProUGUI HealthValueText;
	public TextMeshProUGUI AmmoValueText;
	public TextMeshProUGUI WeaponText;
	public TextMeshProUGUI TargetEnemyNameText;
	public TextMeshProUGUI TargetEnemyHealthText;

	public Image TargetReticle;
	public RectTransform PotentialTargetReticlesContainer;
	[ReadOnly] public RectTransform[] PotentialTargetReticles;
	[ReadOnly] public AIController TargetedEnemy;

	public VehicleController PlayerVehicle;
	public PlayerCamera PlayerCamera;

	private VehicleController.WeaponType currentWeaponType;
	private EquippableWeapon currentWeapon;

	private void Awake()
	{
		var reticles = this.PotentialTargetReticlesContainer.GetComponentsInChildren<RectTransform>().Skip(1).ToArray(); //Skip parent
		this.PotentialTargetReticles = new RectTransform[reticles.Length];
		for (int i = 0; i < reticles.Length; i++)
		{
			this.PotentialTargetReticles[i] = reticles[i];
			this.PotentialTargetReticles[i].gameObject.SetActive(false);
		}
	}

	void Start()
	{
		PlayerCamera = GameObject.FindGameObjectWithTag(Helpers.Tags.PlayerCamera).GetComponent<PlayerCamera>();

		WeaponText.text = PlayerVehicle.EquippedWeaponType.ToString();
		Crosshair.sprite = CrosshairImages[(int)PlayerVehicle.EquippedWeaponType];
		currentWeaponType = PlayerVehicle.EquippedWeaponType;

		ClearTarget();
	}

	void Update()
	{
		JumpBoostValueText.text = Mathf.RoundToInt(PlayerVehicle.CurrentJumpBoost / PlayerVehicle.MaxJumpBoost * 100).ToString();
		ShipSpeedValueText.text = $"{Mathf.RoundToInt(PlayerVehicle.Speed)}KPH";
		HealthValueText.text = Mathf.RoundToInt(PlayerVehicle.CurrentHealth).ToString();

		if (currentWeaponType != PlayerVehicle.EquippedWeaponType)
		{
			currentWeaponType = PlayerVehicle.EquippedWeaponType;
			currentWeapon = PlayerVehicle.EquippedWeapon;
			WeaponText.text = currentWeapon.Name;
			Crosshair.sprite = CrosshairImages[(int)PlayerVehicle.EquippedWeaponType];
		}

		AmmoValueText.text = PlayerVehicle.EquippedWeapon.WeaponText;

		if (TargetedEnemy != null && PlayerCamera.LockOnTarget == null)
		{
			ClearTarget();
		}
		else if (TargetedEnemy != PlayerCamera.LockOnTarget && PlayerCamera.LockOnTarget != null)
		{
			AcquireTarget(PlayerCamera.LockOnTarget);
		}

		if (TargetedEnemy != null)
		{
			var rect = GUIRectWithObject(TargetedEnemy);
			var reticlePosition = PlayerCamera.Camera.WorldToScreenPoint(TargetedEnemy.CentreOfMass);
			TargetReticle.rectTransform.anchoredPosition = new Vector2((int)reticlePosition.x, (int)reticlePosition.y);
			TargetReticle.rectTransform.sizeDelta = new Vector2((int)rect.width, (int)rect.height);
			TargetEnemyHealthText.text = TargetedEnemy.CurrentHealth.ToString();
		}

		var visibleEnemies = PlayerCamera.VisibleTargetsOnScreen.Take(PotentialTargetReticles.Length).ToArray();
		for (int i = 0; i < PotentialTargetReticles.Length; i++)
		{
			var reticle = PotentialTargetReticles[i];
			var targetEnemy = i < visibleEnemies.Length ? visibleEnemies[i] : null;

			if (targetEnemy != null && targetEnemy != TargetedEnemy)
			{
				if (!reticle.gameObject.activeSelf)
					reticle.gameObject.SetActive(true);

				var rect = GUIRectWithObject(visibleEnemies[i]);
				var reticlePosition = PlayerCamera.Camera.WorldToScreenPoint(visibleEnemies[i].CentreOfMass);
				reticle.anchoredPosition = new Vector2((int)reticlePosition.x, (int)reticlePosition.y);
				reticle.sizeDelta = new Vector2((int)rect.width, (int)rect.height);
			}
			else if (reticle.gameObject.activeSelf)
				reticle.gameObject.SetActive(false);
		}
	}

	public void AcquireTarget(AIController target)
	{
		TargetedEnemy = target;
		TargetedEnemy = target.GetComponent<AIController>();

		TargetEnemyNameText.text = target.EnemyTypeName;
		TargetEnemyNameText.gameObject.SetActive(true);

		TargetEnemyHealthText.text = target.CurrentHealth.ToString();
		TargetEnemyHealthText.gameObject.SetActive(true);

		var rect = GUIRectWithObject(target);
		var reticlePosition = PlayerCamera.Camera.WorldToScreenPoint(TargetedEnemy.CentreOfMass);
		TargetReticle.rectTransform.anchoredPosition = new Vector2((int)reticlePosition.x, (int)reticlePosition.y);
		TargetReticle.rectTransform.sizeDelta = new Vector2((int)rect.width, (int)rect.height);
		TargetReticle.gameObject.SetActive(true);
	}

	public void ClearTarget()
	{
		TargetEnemyNameText.text = "";
		TargetEnemyNameText.gameObject.SetActive(false);

		TargetEnemyHealthText.text = "";
		TargetEnemyHealthText.gameObject.SetActive(false);

		TargetedEnemy = null;
		TargetReticle.gameObject.SetActive(false);
	}

	private Rect GUIRectWithObject(AIController enemy)
	{
		Vector3 centre = enemy.Hurtbox.bounds.center;
		Vector3 extents = enemy.Hurtbox.bounds.extents * ReticleScaleFactor;

		//TODO: If we want to have multiple hurtboxes, then we should make the bounds and extents the Min and Max positions for all hurtboxes
		Vector2[] extentPoints = new Vector2[8]
		{
			PlayerCamera.Camera.WorldToScreenPoint(new Vector3(centre.x-extents.x, centre.y-extents.y, centre.z-extents.z)),
			PlayerCamera.Camera.WorldToScreenPoint(new Vector3(centre.x+extents.x, centre.y-extents.y, centre.z-extents.z)),
			PlayerCamera.Camera.WorldToScreenPoint(new Vector3(centre.x-extents.x, centre.y-extents.y, centre.z+extents.z)),
			PlayerCamera.Camera.WorldToScreenPoint(new Vector3(centre.x+extents.x, centre.y-extents.y, centre.z+extents.z)),
			PlayerCamera.Camera.WorldToScreenPoint(new Vector3(centre.x-extents.x, centre.y+extents.y, centre.z-extents.z)),
			PlayerCamera.Camera.WorldToScreenPoint(new Vector3(centre.x+extents.x, centre.y+extents.y, centre.z-extents.z)),
			PlayerCamera.Camera.WorldToScreenPoint(new Vector3(centre.x-extents.x, centre.y+extents.y, centre.z+extents.z)),
			PlayerCamera.Camera.WorldToScreenPoint(new Vector3(centre.x+extents.x, centre.y+extents.y, centre.z+extents.z))
		};
		Vector2 min = extentPoints[0];
		Vector2 max = extentPoints[0];
		foreach (Vector2 v in extentPoints)
		{
			min = Vector2.Min(min, v);
			max = Vector2.Max(max, v);
		}
		return new Rect(min.x, min.y, max.x - min.x, max.y - min.y);
	}
}
