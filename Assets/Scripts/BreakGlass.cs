using UnityEngine;

public class BreakOnImpact : MonoBehaviour
{
    [Tooltip("Prefab that contains all the shards (Breakable_Glass).")]
    public GameObject brokenPrefab;

    [Tooltip("Minimum collision speed needed to break.")]
    public float breakVelocity = 2f;

    [Header("Explosion settings")]
    [Tooltip("Force de l'explosion appliquée aux morceaux.")]
    public float explosionForce = 0.5f;

    [Tooltip("Rayon de l'explosion autour du point d'impact.")]
    public float explosionRadius = 0.3f;

    [Tooltip("Force vers le haut ajoutée à l'explosion.")]
    public float upwardsModifier = 0f;
    
    [Header("Shard Settings")]
    [Tooltip("Détruit les shards après X secondes (0 = jamais)")]
    public float shardLifetime = 0f;

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.relativeVelocity.magnitude < breakVelocity)
            return;

        GameObject broken = Instantiate(
            brokenPrefab,
            transform.position,
            transform.rotation
        );

        Rigidbody rb = GetComponent<Rigidbody>();

        foreach (Rigidbody childRb in broken.GetComponentsInChildren<Rigidbody>())
        {
            if (rb != null)
                childRb.linearVelocity = rb.linearVelocity;

            childRb.AddExplosionForce(
                explosionForce,
                transform.position,
                explosionRadius,
                upwardsModifier,
                ForceMode.Impulse
            );
        
            // Détache chaque Cylinder_cell du parent Breakable_Glass
            childRb.transform.SetParent(null);
            
            // Optionnel : auto-détruit après 10 secondes
            if (shardLifetime > 0)
            {
                Destroy(childRb.gameObject, shardLifetime);
            }
        }

        // Détruit le parent Breakable_Glass vide
        Destroy(broken, 0.1f);
        
        // Détruit le verre intact original
        Destroy(gameObject);
    }
}