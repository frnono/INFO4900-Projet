using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Système de déformation physiquement réaliste pour une balle souple
/// Implémente le squash & stretch avec conservation du volume
/// Conçu pour le projet INFO4900 - Challenge de dynamique physique
/// </summary>
[RequireComponent(typeof(MeshFilter), typeof(Rigidbody))]
public class DeformableBall : MonoBehaviour
{
    [Header("Paramètres Physiques de la Balle")]
    [Tooltip("Module de Young - Rigidité du matériau (N/m²)")]
    [Range(1000f, 50000f)]
    public float youngModulus = 15000f;

    [Space(10)]
    [Header("Paramètres de Déformation")]
    [Tooltip("Intensité maximale de déformation (0-1)")]
    [Range(0f, 1f)]
    public float maxDeformation = 0.5f;

    [Tooltip("Vitesse de récupération de la forme originale")]
    [Range(1f, 30f)]
    public float recoverySpeed = 12f;

    [Tooltip("Force minimale pour déclencher la déformation (N)")]
    public float forceThreshold = 3f;

    [Tooltip("Rayon d'influence de la déformation (multiplicateur du rayon original)")]
    [Range(1f, 3f)]
    public float influenceRadiusMultiplier = 1.5f;

    [Tooltip("Coefficient de Poisson - Conservation du volume (0.3-0.5 pour caoutchouc)")]
    [Range(0.3f, 0.499f)]
    public float poissonRatio = 0.45f;

    [Space(10)]
    [Header("Amortissement et Stabilité")]
    [Tooltip("Amortissement des oscillations de la déformation (0-1)")]
    [Range(0.8f, 0.99f)]
    public float dampingFactor = 0.95f;

    [Tooltip("Multiplicateur de vitesse initiale de récupération")]
    [Range(0.5f, 3f)]
    public float snapbackMultiplier = 1.2f;

    [Tooltip("Exposant de la courbe de récupération (>1 = récupération initiale plus rapide)")]
    [Range(1f, 3f)]
    public float recoveryExponent = 1.5f;

    [Tooltip("Empêcher l'extension au-delà de la forme originale")]
    public bool preventOvershoot = true;

    [Space(10)]
    [Header("Qualité de Simulation")]
    [Tooltip("Nombre de substeps par frame pour meilleure stabilité")]
    [Range(1, 5)]
    public int substepsPerFrame = 2;

    [Tooltip("Seuil de convergence pour déformation inactive")]
    [Range(0.0001f, 0.01f)]
    public float convergenceThreshold = 0.001f;

    [Space(10)]
    [Header("Visualisation Debug")]
    [Tooltip("Afficher les gizmos de déformation")]
    public bool showDebugGizmos = true;

    [Tooltip("Afficher les statistiques de performance")]
    public bool showPerformanceStats = false;

    // Components
    private Rigidbody rb;
    private MeshFilter meshFilter;
    private Mesh deformableMesh;

    // Mesh data
    private Vector3[] originalVertices;
    private Vector3[] currentVertices;
    private Vector3[] vertexVelocities;
    private float[] vertexDeformations;

    // Performance optimization - track only active vertices
    private List<int> activeVertexIndices = new List<int>(128);
    private HashSet<int> activeVertexSet = new HashSet<int>();
    private List<int> verticesToRemove = new List<int>(64); // Réutilisé à chaque frame

    // Impact state
    private Vector3 lastImpactPoint;
    private Vector3 lastImpactNormal;
    private Vector3 lastRelativeVelocity;
    private float impactForce = 0f;
    private bool isDeforming = false;
    private float deformationTimer = 0f;

    // Cached values
    private float originalRadius;
    private Vector3 originalScale;
    private float influenceRadius;
    private float stiffnessConstant;
    private bool needsNormalRecalculation = false;

    // Performance stats
    private int lastActiveVertexCount = 0;
    private float lastUpdateTime = 0f;

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
        
        // Pre-allocate capacity based on expected active vertices
        int expectedActive = Mathf.Min(originalVertices.Length / 4, 256);
        activeVertexIndices = new List<int>(expectedActive);
        activeVertexSet = new HashSet<int>();
        verticesToRemove = new List<int>(expectedActive / 2);
    }

    void ConfigurePhysics()
    {
        if (rb.collisionDetectionMode == CollisionDetectionMode.Discrete)
        {
            rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
        }
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
        // Recalculer le rayon d'influence quand le paramètre change dans l'éditeur
        if (originalRadius > 0f)
        {
            UpdateInfluenceRadius();
        }
    }

    void FixedUpdate()
    {
        if (isDeforming)
        {
            float startTime = Time.realtimeSinceStartup;
            
            needsNormalRecalculation = false;
            
            // Substeps for better numerical stability
            float subDeltaTime = Time.fixedDeltaTime / substepsPerFrame;
            for (int step = 0; step < substepsPerFrame; step++)
            {
                UpdateDeformationSubstep(subDeltaTime);
            }
            
            // Recalculer les normales une seule fois après tous les substeps
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

        // Calculate relative velocity properly (account for other object's velocity)
        Vector3 otherVelocity = collision.rigidbody != null ? 
            collision.rigidbody.linearVelocity : Vector3.zero;
        lastRelativeVelocity = rb.linearVelocity - otherVelocity;
        
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
        
        // Clear previous active vertices
        activeVertexIndices.Clear();
        activeVertexSet.Clear();

        float normalizedForce = Mathf.Clamp01(impactForce / (rb.mass * 20f));
        float influenceRadiusSqr = influenceRadius * influenceRadius; // Optimisation

        for (int i = 0; i < originalVertices.Length; i++)
        {
            Vector3 vertex = originalVertices[i];
            Vector3 toVertex = vertex - lastImpactPoint;
            float distanceSqrToImpact = toVertex.sqrMagnitude;
            
            // Early rejection avec distance au carré (évite sqrt)
            if (distanceSqrToImpact > influenceRadiusSqr)
            {
                currentVertices[i] = vertex;
                vertexDeformations[i] = 0f;
                continue;
            }

            float distanceToImpact = Mathf.Sqrt(distanceSqrToImpact);
            float spatialInfluence = CalculateSpatialInfluence(vertex, distanceToImpact);

            if (spatialInfluence > 0.01f)
            {
                float compressionAmount = normalizedForce * maxDeformation * spatialInfluence;
                vertexDeformations[i] = compressionAmount;

                Vector3 deformation = CalculateVolumeConservingDeformation(
                    vertex,
                    lastImpactNormal,
                    compressionAmount
                );

                currentVertices[i] = vertex + deformation;
                vertexVelocities[i] = -deformation * recoverySpeed * snapbackMultiplier;
                
                // Add to active list
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
        float distanceFactor = Mathf.Max(0f, 1f - (distanceToImpact / influenceRadius));

        Vector3 vertexDirection = vertex.normalized;
        float angleFactor = Mathf.Max(0f, Vector3.Dot(vertexDirection, lastImpactNormal));

        float influence = distanceFactor * angleFactor;
        return Mathf.Pow(influence, 1.5f);
    }

    Vector3 CalculateVolumeConservingDeformation(Vector3 vertex, Vector3 impactNormal, float compression)
    {
        // Compression along the impact normal
        Vector3 normalDeformation = -impactNormal * compression;

        // Find the radial component of the vertex (perpendicular to impact normal)
        // This is the key to proper volume conservation - expand in ALL perpendicular directions
        Vector3 radialDirection = Vector3.ProjectOnPlane(vertex, impactNormal);

        // If the vertex is on or very close to the impact axis, no radial expansion needed
        if (radialDirection.sqrMagnitude < 0.0001f)
        {
            return normalDeformation;
        }

        // Normalize to get the radial direction
        float radialDistance = radialDirection.magnitude;
        radialDirection /= radialDistance;

        // Radial expansion proportional to compression and Poisson ratio
        // Poisson's ratio defines: transverse_strain = -ν * axial_strain
        // For volume conservation, perpendicular directions expand when compressed
        float radialExpansion = compression * poissonRatio;
        Vector3 radialDeformation = radialDirection * radialExpansion;

        return normalDeformation + radialDeformation;
    }

    void UpdateDeformationSubstep(float deltaTime)
    {
        bool hasActiveDeformation = false;
        verticesToRemove.Clear(); // Réutiliser la liste au lieu d'en créer une nouvelle

        // Only update active vertices
        for (int idx = 0; idx < activeVertexIndices.Count; idx++)
        {
            int i = activeVertexIndices[idx];
            
            if (vertexDeformations[i] > convergenceThreshold)
            {
                hasActiveDeformation = true;

                Vector3 displacement = currentVertices[i] - originalVertices[i];
                float displacementMag = displacement.magnitude;
                
                // Safety check
                if (displacementMag < 0.0001f)
                {
                    verticesToRemove.Add(i);
                    continue;
                }
                
                float exponentialFactor = Mathf.Pow(
                    displacementMag / (originalRadius * 0.5f), 
                    recoveryExponent - 1f
                );
                Vector3 restoreForce = -displacement * stiffnessConstant * exponentialFactor;

                vertexVelocities[i] += restoreForce * deltaTime;
                vertexVelocities[i] *= dampingFactor;

                Vector3 newPosition = currentVertices[i] + vertexVelocities[i] * deltaTime;

                // Prevent overshoot simplifié
                if (preventOvershoot)
                {
                    Vector3 originalVertex = originalVertices[i];
                    float originalDistSqr = originalVertex.sqrMagnitude;
                    
                    // Safety check for vertices near center
                    if (originalDistSqr > 0.0001f)
                    {
                        float newDistSqr = newPosition.sqrMagnitude;

                        // Si le vertex s'éloigne plus que la position originale
                        if (newDistSqr > originalDistSqr * 1.01f) // 1% de tolérance
                        {
                            // Clamper à la distance originale
                            float originalDist = Mathf.Sqrt(originalDistSqr);
                            newPosition = newPosition.normalized * originalDist;
                            vertexVelocities[i] *= 0.3f;
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
                // Smoothly return to rest
                currentVertices[i] = Vector3.Lerp(
                    currentVertices[i],
                    originalVertices[i],
                    deltaTime * recoverySpeed * 3f
                );

                vertexVelocities[i] = Vector3.zero;
                vertexDeformations[i] = 0f;
                
                // Mark for removal from active list
                verticesToRemove.Add(i);
                needsNormalRecalculation = true;
            }
        }

        // Remove converged vertices from active list
        foreach (int vertexIndex in verticesToRemove)
        {
            activeVertexSet.Remove(vertexIndex);
        }

        // Manual removal to avoid lambda allocation
        for (int i = activeVertexIndices.Count - 1; i >= 0; i--)
        {
            if (!activeVertexSet.Contains(activeVertexIndices[i]))
            {
                activeVertexIndices.RemoveAt(i);
            }
        }
        
        lastActiveVertexCount = activeVertexIndices.Count;

        // Mesh update is deferred to FixedUpdate -> UpdateMeshGeometry
        // to avoid multiple updates per frame

        if (!hasActiveDeformation || deformationTimer > 3f)
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
        // Only recalculate tangents if needed for your materials
        // deformableMesh.RecalculateTangents();
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

    // Public properties
    public bool IsDeforming => isDeforming;
    public float CurrentImpactForce => impactForce;
    public float DeformationProgress => deformationTimer;
    public int ActiveVertexCount => lastActiveVertexCount;
    public float LastUpdateTimeMs => lastUpdateTime;
    public float CurrentInfluenceRadius => influenceRadius;

    void OnDrawGizmos()
    {
        if (!showDebugGizmos || !isDeforming || !Application.isPlaying) return;

        Gizmos.color = Color.red;
        Vector3 worldImpactPoint = transform.TransformPoint(lastImpactPoint);
        Gizmos.DrawWireSphere(worldImpactPoint, 0.05f);

        Gizmos.color = Color.yellow;
        Vector3 worldNormal = transform.TransformDirection(lastImpactNormal);
        Gizmos.DrawLine(worldImpactPoint, worldImpactPoint + worldNormal * 0.3f);

        Gizmos.color = new Color(1f, 0f, 0f, 0.3f);
        Gizmos.DrawWireSphere(worldImpactPoint, influenceRadius * transform.localScale.x);
    }

    void OnDrawGizmosSelected()
    {
        if (!Application.isPlaying || currentVertices == null)
            return;

        if (isDeforming)
        {
            Gizmos.color = new Color(1f, 0.6f, 0f, 0.15f);
            Vector3 worldImpact = transform.TransformPoint(lastImpactPoint);
            Gizmos.DrawSphere(worldImpact, influenceRadius * transform.localScale.x);

            Gizmos.color = Color.magenta;
            Vector3 worldImpactNormal = transform.TransformDirection(lastImpactNormal);
            Gizmos.DrawLine(worldImpact, worldImpact + worldImpactNormal * 0.3f * transform.localScale.x);

            for (int i = 0; i < currentVertices.Length; i++)
            {
                float deformation = Vector3.Distance(currentVertices[i], originalVertices[i]);
                if (deformation > 0.0005f)
                {
                    Vector3 from = transform.TransformPoint(originalVertices[i]);
                    Vector3 to = transform.TransformPoint(currentVertices[i]);

                    float maxDef = originalRadius * 0.30f;
                    float t = Mathf.Clamp01(deformation / maxDef);

                    Color gradientColor;
                    if (t < 0.33f)
                        gradientColor = Color.Lerp(Color.blue, Color.green, t / 0.33f);
                    else if (t < 0.66f)
                        gradientColor = Color.Lerp(Color.green, Color.yellow, (t - 0.33f) / 0.33f);
                    else
                        gradientColor = Color.Lerp(Color.yellow, Color.red, (t - 0.66f) / 0.34f);

                    Gizmos.color = gradientColor;
                    Gizmos.DrawSphere(to, 0.0125f * transform.localScale.x);

                    Gizmos.color = new Color(gradientColor.r, gradientColor.g, gradientColor.b, 0.6f);
                    Gizmos.DrawLine(from, to);
                }
            }
        }

        // Arêtes du mesh (optionnel: fil de fer)
        if (deformableMesh != null && deformableMesh.triangles.Length > 0)
        {
            Gizmos.color = new Color(0f,0f,0f,0.15f);
            int[] tris = deformableMesh.triangles;
            for (int i = 0; i < tris.Length; i += 3)
            {
                Vector3 v0 = transform.TransformPoint(currentVertices[tris[i]]);
                Vector3 v1 = transform.TransformPoint(currentVertices[tris[i+1]]);
                Vector3 v2 = transform.TransformPoint(currentVertices[tris[i+2]]);
                Gizmos.DrawLine(v0, v1);
                Gizmos.DrawLine(v1, v2);
                Gizmos.DrawLine(v2, v0);
            }
        }
    }

    void OnGUI()
    {
        if (!showPerformanceStats || !Application.isPlaying) return;

        GUILayout.BeginArea(new Rect(10, 10, 300, 170));
        GUILayout.Box("Deformable Ball Stats");
        GUILayout.Label($"Active Vertices: {lastActiveVertexCount} / {originalVertices?.Length ?? 0}");
        GUILayout.Label($"Update Time: {lastUpdateTime:F3} ms");
        GUILayout.Label($"Is Deforming: {isDeforming}");
        GUILayout.Label($"Impact Force: {impactForce:F1} N");
        GUILayout.Label($"Deformation Time: {deformationTimer:F2} s");
        GUILayout.Label($"Influence Radius: {influenceRadius:F3}");
        GUILayout.EndArea();
    }
}