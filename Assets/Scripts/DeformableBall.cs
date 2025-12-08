using UnityEngine;
using System.Collections.Generic;

// Déformation physique avec conservation du volume (squash & stretch)
[RequireComponent(typeof(MeshFilter), typeof(Rigidbody))]
public class DeformableBall : MonoBehaviour
{
    public enum MaterialPreset
    {
        Custom,
        Souple
    }

    [Header("Presets de Matériaux")]
    [Tooltip("Sélectionner un preset ou 'Custom' pour ajuster manuellement")]
    public MaterialPreset materialPreset = MaterialPreset.Custom;

    [Space(10)]
    [Header("Paramètres Physiques")]
    [Tooltip("Rigidité du matériau")]
    [Range(0f, 25000f)]
    public float youngModulus = 15000f;

    [Space(10)]
    [Header("Déformation")]
    [Tooltip("Intensité maximale (0-1)")]
    [Range(0f, 1f)]
    public float maxDeformation = 0.5f;

    [Tooltip("Vitesse de récupération")]
    [Range(1f, 30f)]
    public float recoverySpeed = 12f;

    [Tooltip("Force minimale pour déformer")]
    public float forceThreshold = 3f;

    [Tooltip("Sensibilité aux impacts")]
    [Range(1f, 20f)]
    public float forceNormalizationDivisor = 5f;

    [Tooltip("Rayon d'influence (x rayon original)")]
    [Range(1f, 5f)]
    public float influenceRadiusMultiplier = 1.5f;

    [Tooltip("Conservation du volume (0.3-0.5)")]
    [Range(0.3f, 0.499f)]
    public float poissonRatio = 0.45f;

    [Space(10)]
    [Header("Stabilité")]
    [Tooltip("Amortissement des oscillations")]
    [Range(0.8f, 0.99f)]
    public float dampingFactor = 0.95f;

    [Tooltip("Multiplicateur vitesse de récupération")]
    [Range(0.5f, 3f)]
    public float snapbackMultiplier = 1.2f;

    [Tooltip("Courbe de récupération (>1 = plus rapide au début)")]
    [Range(1f, 3f)]
    public float recoveryExponent = 1.5f;

    [Tooltip("Empêcher l'extension au-delà de la forme originale")]
    public bool preventOvershoot = true;

    [Space(10)]
    [Header("Paramètres Avancés")]
    [Tooltip("Seuil minimal d'influence")]
    [Range(0.001f, 0.1f)]
    public float minSpatialInfluence = 0.01f;

    [Tooltip("Concentration de la déformation")]
    [Range(1f, 3f)]
    public float spatialInfluenceExponent = 1.5f;

    [Tooltip("Normalisation du rayon")]
    [Range(0.1f, 1f)]
    public float radiusNormalizationFactor = 0.5f;

    [Tooltip("Vitesse retour final")]
    [Range(1f, 10f)]
    public float finalRecoverySpeedMultiplier = 3f;

    [Tooltip("Tolérance overshoot (1.0 = aucune)")]
    [Range(1f, 1.1f)]
    public float overshootTolerance = 1.01f;

    [Tooltip("Réduction vélocité overshoot")]
    [Range(0.1f, 0.9f)]
    public float overshootVelocityDamping = 0.3f;

    [Space(10)]
    [Header("Qualité")]
    [Tooltip("Substeps par frame")]
    [Range(1, 5)]
    public int substepsPerFrame = 2;

    [Tooltip("Seuil de convergence")]
    [Range(0.0001f, 0.01f)]
    public float convergenceThreshold = 0.001f;

    [Tooltip("Temps max avant reset (s)")]
    [Range(2f, 10f)]
    public float maxDeformationTime = 5f;

    [Space(10)]
    [Header("Debug")]
    public bool showDebugGizmos = true;
    public bool showPerformanceStats = false;

    private Rigidbody rb;
    private MeshFilter meshFilter;
    private Mesh deformableMesh;

    private Vector3[] originalVertices;
    private Vector3[] currentVertices;
    private Vector3[] vertexVelocities;
    private float[] vertexDeformations;

    // Optimisation: tracking vertices actifs uniquement
    private List<int> activeVertexIndices = new List<int>(128);
    private HashSet<int> activeVertexSet = new HashSet<int>();
    private List<int> verticesToRemove = new List<int>(64);

    private Vector3 lastImpactPoint;
    private Vector3 lastImpactNormal;
    private Vector3 lastRelativeVelocity;
    private float impactForce = 0f;
    private bool isDeforming = false;
    private float deformationTimer = 0f;

    private float originalRadius;
    private Vector3 originalScale;
    private float influenceRadius;
    private float stiffnessConstant;
    private bool needsNormalRecalculation = false;

    private int lastActiveVertexCount = 0;
    private float lastUpdateTime = 0f;
    private MaterialPreset lastAppliedPreset = MaterialPreset.Custom;

    void Start()
    {
        InitializeComponents();
        InitializeMesh();
        ConfigurePhysics();
        CalculateOriginalRadius();
        CacheConstants();
    }

    void InitializeComponents()
    {
        rb = GetComponent<Rigidbody>();
        meshFilter = GetComponent<MeshFilter>();

        deformableMesh = Instantiate(meshFilter.sharedMesh);
        deformableMesh.name = "Deformable Ball Mesh";
        meshFilter.mesh = deformableMesh;
    }

    void InitializeMesh()
    {
        originalVertices = deformableMesh.vertices;
        currentVertices = new Vector3[originalVertices.Length];
        vertexVelocities = new Vector3[originalVertices.Length];
        vertexDeformations = new float[originalVertices.Length];

        System.Array.Copy(originalVertices, currentVertices, originalVertices.Length);

        int expectedActive = Mathf.Min(originalVertices.Length / 4, 256);
        activeVertexIndices = new List<int>(expectedActive);
        activeVertexSet = new HashSet<int>();
        verticesToRemove = new List<int>(expectedActive / 2);
    }

    void ConfigurePhysics()
    {
        // Continuous collision pour détecter les impacts à haute vitesse
        if (rb.collisionDetectionMode == CollisionDetectionMode.Discrete)
        {
            rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
        }
        // Interpolation pour un mouvement fluide
        if (rb.interpolation == RigidbodyInterpolation.None)
        {
            rb.interpolation = RigidbodyInterpolation.Interpolate;
        }
    }

    void CalculateOriginalRadius()
    {
        float maxDist = 0f;
        foreach (Vector3 vertex in originalVertices)
        {
            float dist = vertex.magnitude;
            if (dist > maxDist) maxDist = dist;
        }
        originalRadius = maxDist;
        originalScale = transform.localScale;
        UpdateInfluenceRadius();
    }

    void UpdateInfluenceRadius()
    {
        influenceRadius = originalRadius * influenceRadiusMultiplier;
    }

    void CacheConstants()
    {
        stiffnessConstant = (youngModulus / 10000f) * recoverySpeed;
    }

    void OnValidate()
    {
        if (originalRadius > 0f)
        {
            UpdateInfluenceRadius();
        }

        if (materialPreset != MaterialPreset.Custom && materialPreset != lastAppliedPreset)
        {
            ApplyMaterialPreset(materialPreset);
        }
    }

    [ContextMenu("Apply Current Preset")]
    public void ApplyMaterialPreset(MaterialPreset preset)
    {
        lastAppliedPreset = preset;

        switch (preset)
        {
            case MaterialPreset.Souple:
                youngModulus = 500f;
                maxDeformation = 0.75f;
                recoverySpeed = 8f;
                forceThreshold = 0.25f;
                forceNormalizationDivisor = 10f;
                influenceRadiusMultiplier = 5f;
                poissonRatio = 0.46f;

                dampingFactor = 0.92f;
                snapbackMultiplier = 1.8f;
                recoveryExponent = 2.0f;
                preventOvershoot = true;

                minSpatialInfluence = 0.035f;
                spatialInfluenceExponent = 1.35f;
                radiusNormalizationFactor = 0.6f;
                finalRecoverySpeedMultiplier = 4f;
                overshootTolerance = 1.02f;
                overshootVelocityDamping = 0.25f;

                substepsPerFrame = 3;
                maxDeformationTime = 6f;

                Debug.Log("Preset appliqué: Souple");
                break;

            case MaterialPreset.Custom:
                Debug.Log("Mode Custom: ajustez les paramètres manuellement");
                break;
        }

        if (Application.isPlaying)
        {
            UpdateInfluenceRadius();
            CacheConstants();
        }
    }

    [ContextMenu("Preset: Souple")]
    void ApplySouple() { materialPreset = MaterialPreset.Souple; ApplyMaterialPreset(materialPreset); }

    void FixedUpdate()
    {
        if (isDeforming)
        {
            float startTime = Time.realtimeSinceStartup;

            needsNormalRecalculation = false;

            float subDeltaTime = Time.fixedDeltaTime / substepsPerFrame;
            for (int step = 0; step < substepsPerFrame; step++)
            {
                UpdateDeformationSubstep(subDeltaTime);
            }

            if (needsNormalRecalculation)
            {
                UpdateMeshGeometry();
            }

            deformationTimer += Time.fixedDeltaTime;

            if (showPerformanceStats)
            {
                lastUpdateTime = (Time.realtimeSinceStartup - startTime) * 1000f;
            }
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        ProcessImpact(collision);
    }

    void ProcessImpact(Collision collision)
    {
        if (collision.contacts.Length == 0) return;

        ContactPoint contact = collision.contacts[0];

        // Calcul de la vélocité relative pour gérer les objets en mouvement
        Vector3 otherVelocity = collision.rigidbody != null ?
            collision.rigidbody.linearVelocity : Vector3.zero;
        lastRelativeVelocity = rb.linearVelocity - otherVelocity;

        // Force d'impact basée sur la composante normale de la vélocité
        Vector3 normalVelocity = Vector3.Project(lastRelativeVelocity, contact.normal);
        impactForce = normalVelocity.magnitude * rb.mass;

        if (impactForce >= forceThreshold)
        {
            lastImpactPoint = transform.InverseTransformPoint(contact.point);
            lastImpactNormal = transform.InverseTransformDirection(-contact.normal);

            InitiateDeformation();
        }
    }

    void InitiateDeformation()
    {
        isDeforming = true;
        deformationTimer = 0f;

        activeVertexIndices.Clear();
        activeVertexSet.Clear();

        // Normaliser la force d'impact pour contrôler l'intensité de déformation
        float normalizedForce = Mathf.Clamp01(impactForce / (rb.mass * forceNormalizationDivisor));
        float influenceRadiusSqr = influenceRadius * influenceRadius;

        // Appliquer la déformation à chaque vertex selon sa proximité à l'impact
        for (int i = 0; i < originalVertices.Length; i++)
        {
            Vector3 vertex = originalVertices[i];
            Vector3 toVertex = vertex - lastImpactPoint;
            float distanceSqrToImpact = toVertex.sqrMagnitude;

            // Early rejection (évite sqrt)
            if (distanceSqrToImpact > influenceRadiusSqr)
            {
                currentVertices[i] = vertex;
                vertexDeformations[i] = 0f;
                continue;
            }

            float distanceToImpact = Mathf.Sqrt(distanceSqrToImpact);
            float spatialInfluence = CalculateSpatialInfluence(vertex, distanceToImpact);

            if (spatialInfluence > minSpatialInfluence)
            {
                float compressionAmount = normalizedForce * maxDeformation * spatialInfluence;
                vertexDeformations[i] = compressionAmount;

                // Déformation avec conservation du volume (Poisson)
                Vector3 deformation = CalculateVolumeConservingDeformation(
                    vertex,
                    lastImpactNormal,
                    compressionAmount
                );

                currentVertices[i] = vertex + deformation;
                vertexVelocities[i] = -deformation * recoverySpeed * snapbackMultiplier;

                activeVertexIndices.Add(i);
                activeVertexSet.Add(i);
            }
            else
            {
                currentVertices[i] = vertex;
                vertexDeformations[i] = 0f;
            }
        }

        lastActiveVertexCount = activeVertexIndices.Count;
        UpdateMeshGeometry();
    }

    float CalculateSpatialInfluence(Vector3 vertex, float distanceToImpact)
    {
        // Influence diminue avec la distance
        float distanceFactor = Mathf.Max(0f, 1f - (distanceToImpact / influenceRadius));

        // Influence plus forte pour vertices alignés avec la normale d'impact
        Vector3 vertexDirection = vertex.normalized;
        float angleFactor = Mathf.Max(0f, Vector3.Dot(vertexDirection, lastImpactNormal));

        float influence = distanceFactor * angleFactor;
        return Mathf.Pow(influence, spatialInfluenceExponent);
    }

    Vector3 CalculateVolumeConservingDeformation(Vector3 vertex, Vector3 impactNormal, float compression)
    {
        Vector3 normalDeformation = -impactNormal * compression;

        // Expansion radiale pour conserver le volume (ratio de Poisson)
        Vector3 radialDirection = Vector3.ProjectOnPlane(vertex, impactNormal);

        if (radialDirection.sqrMagnitude < 0.0001f)
        {
            return normalDeformation;
        }

        float radialDistance = radialDirection.magnitude;
        radialDirection /= radialDistance;

        float radialExpansion = compression * poissonRatio;
        Vector3 radialDeformation = radialDirection * radialExpansion;

        return normalDeformation + radialDeformation;
    }

    void UpdateDeformationSubstep(float deltaTime)
    {
        bool hasActiveDeformation = false;
        verticesToRemove.Clear();

        // Mise à jour des vertices actifs uniquement (optimisation)
        for (int idx = 0; idx < activeVertexIndices.Count; idx++)
        {
            int i = activeVertexIndices[idx];

            if (vertexDeformations[i] > convergenceThreshold)
            {
                hasActiveDeformation = true;

                Vector3 displacement = currentVertices[i] - originalVertices[i];
                float displacementMag = displacement.magnitude;

                if (displacementMag < 0.0001f)
                {
                    verticesToRemove.Add(i);
                    continue;
                }

                // Force de restauration avec courbe exponentielle
                float exponentialFactor = Mathf.Pow(
                    displacementMag / (originalRadius * radiusNormalizationFactor),
                    recoveryExponent - 1f
                );
                Vector3 restoreForce = -displacement * stiffnessConstant * exponentialFactor;

                vertexVelocities[i] += restoreForce * deltaTime;
                vertexVelocities[i] *= dampingFactor;

                Vector3 newPosition = currentVertices[i] + vertexVelocities[i] * deltaTime;

                // Empêcher le vertex de dépasser sa position originale
                if (preventOvershoot)
                {
                    Vector3 originalVertex = originalVertices[i];
                    float originalDistSqr = originalVertex.sqrMagnitude;

                    if (originalDistSqr > 0.0001f)
                    {
                        float newDistSqr = newPosition.sqrMagnitude;

                        if (newDistSqr > originalDistSqr * overshootTolerance)
                        {
                            float originalDist = Mathf.Sqrt(originalDistSqr);
                            newPosition = newPosition.normalized * originalDist;
                            vertexVelocities[i] *= overshootVelocityDamping;
                        }
                    }
                }

                currentVertices[i] = newPosition;

                float distToOriginal = Vector3.Distance(currentVertices[i], originalVertices[i]);
                vertexDeformations[i] = distToOriginal;

                needsNormalRecalculation = true;
            }
            else
            {
                // Retour final progressif vers la forme d'origine
                currentVertices[i] = Vector3.Lerp(
                    currentVertices[i],
                    originalVertices[i],
                    deltaTime * recoverySpeed * finalRecoverySpeedMultiplier
                );

                vertexVelocities[i] = Vector3.zero;
                vertexDeformations[i] = 0f;

                verticesToRemove.Add(i);
                needsNormalRecalculation = true;
            }
        }

        // Nettoyer les vertices convergés de la liste active
        foreach (int vertexIndex in verticesToRemove)
        {
            activeVertexSet.Remove(vertexIndex);
        }

        for (int i = activeVertexIndices.Count - 1; i >= 0; i--)
        {
            if (!activeVertexSet.Contains(activeVertexIndices[i]))
            {
                activeVertexIndices.RemoveAt(i);
            }
        }

        lastActiveVertexCount = activeVertexIndices.Count;

        if (!hasActiveDeformation || deformationTimer > maxDeformationTime)
        {
            ResetToOriginalShape();
        }
    }

    void ResetToOriginalShape()
    {
        isDeforming = false;
        deformationTimer = 0f;
        impactForce = 0f;

        System.Array.Copy(originalVertices, currentVertices, originalVertices.Length);
        System.Array.Clear(vertexVelocities, 0, vertexVelocities.Length);
        System.Array.Clear(vertexDeformations, 0, vertexDeformations.Length);
        
        activeVertexIndices.Clear();
        activeVertexSet.Clear();
        verticesToRemove.Clear();
        lastActiveVertexCount = 0;

        UpdateMeshGeometry();
    }

    void UpdateMeshGeometry()
    {
        deformableMesh.vertices = currentVertices;
        deformableMesh.RecalculateNormals();
        deformableMesh.RecalculateBounds();
    }

    public void LaunchBall(Vector3 force)
    {
        rb.WakeUp();
        rb.AddForce(force, ForceMode.Impulse);
    }

    public void LaunchBall(Vector3 direction, float speed)
    {
        rb.WakeUp();
        rb.linearVelocity = direction.normalized * speed;
    }

    public void ResetBall(Vector3 position)
    {
        transform.position = position;
        transform.rotation = Quaternion.identity;
        transform.localScale = originalScale;

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        rb.WakeUp();

        ResetToOriginalShape();
    }

    public void ResetBall()
    {
        ResetBall(transform.position);
    }

    public bool IsDeforming => isDeforming;
    public float CurrentImpactForce => impactForce;
    public float DeformationProgress => deformationTimer;
    public int ActiveVertexCount => lastActiveVertexCount;
    public float LastUpdateTimeMs => lastUpdateTime;
    public float CurrentInfluenceRadius => influenceRadius;

    void OnDrawGizmos()
    {
        if (!showDebugGizmos || !Application.isPlaying || !isDeforming || currentVertices == null)
            return;

        Vector3 worldImpactPoint = transform.TransformPoint(lastImpactPoint);

        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(worldImpactPoint, 0.05f);

        Gizmos.color = Color.yellow;
        Vector3 worldNormal = transform.TransformDirection(lastImpactNormal);
        Gizmos.DrawLine(worldImpactPoint, worldImpactPoint + worldNormal * 0.3f);

        for (int i = 0; i < currentVertices.Length; i++)
        {
            float deformation = Vector3.Distance(currentVertices[i], originalVertices[i]);
            if (deformation > 0.001f)
            {
                Vector3 from = transform.TransformPoint(originalVertices[i]);
                Vector3 to = transform.TransformPoint(currentVertices[i]);

                float t = Mathf.Clamp01(deformation / (originalRadius * 0.3f));
                Gizmos.color = Color.Lerp(Color.green, Color.red, t);

                Gizmos.DrawLine(from, to);
            }
        }
    }

    void OnGUI()
    {
        if (!showPerformanceStats || !Application.isPlaying) return;

        GUILayout.BeginArea(new Rect(10, 10, 320, 180));
        GUILayout.Box("Deformable Ball Stats");
        GUILayout.Label($"Material Preset: {materialPreset}");
        GUILayout.Label($"Active Vertices: {lastActiveVertexCount} / {originalVertices?.Length ?? 0}");
        GUILayout.Label($"Update Time: {lastUpdateTime:F3} ms");
        GUILayout.Label($"Is Deforming: {isDeforming}");
        GUILayout.Label($"Impact Force: {impactForce:F1} N");
        GUILayout.Label($"Deformation Time: {deformationTimer:F2} s");
        GUILayout.Label($"Influence Radius: {influenceRadius:F3}");
        GUILayout.EndArea();
    }
}
