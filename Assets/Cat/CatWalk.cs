using UnityEngine;

public class CatWalk : MonoBehaviour
{
    [Header("Marche du chat")]
    public Transform walkPoint;      // Destination
    public float walkSpeed = 0.12f;  // Vitesse du chat

    private Animator anim;
    private bool isWalking = true;
    private bool reached = false;

    [Header("Contact avec la balle (basé sur le mesh)")]
    public Transform pawHitPoint;    // Empty sur la patte / tête / poitrine du chat
    public Rigidbody ballRb;         // Rigidbody de la balle
    public float hitDistance = 0.07f; // Distance de contact visuel (essaie 0.05–0.1)
    public float hitForce = 2.5f;    // Force appliquée à la balle
    private bool hasHitBall = false;

    void Start()
    {
        anim = GetComponentInChildren<Animator>();

        if (anim != null)
            anim.SetBool("isWalking", true); // lance animation Walk
    }

    void Update()
    {
        // Vérifie d'abord si on doit frapper la balle
        HandleBallHit();

        if (reached || walkPoint == null) return;

        MoveToPoint();
    }

    void MoveToPoint()
    {
        // Position cible (même hauteur)
        Vector3 target = new Vector3(
            walkPoint.position.x,
            transform.position.y,
            walkPoint.position.z
        );

        // Distance actuelle
        float distance = Vector3.Distance(transform.position, target);

        // Arrête à la cible
        if (distance < 0.01f)
        {
            reached = true;
            isWalking = false;

            // place exactement le chat au point
            transform.position = target;

            if (anim != null)
                anim.SetBool("isWalking", false); // Idle

            return;
        }

        // Marche vers la cible
        float step = walkSpeed * Time.deltaTime;
        transform.position = Vector3.MoveTowards(transform.position, target, step);

        // Rotation douce vers la cible
        Vector3 dir = target - transform.position;
        dir.y = 0f;

        if (dir.sqrMagnitude > 0.0001f)
        {
            Quaternion rot = Quaternion.LookRotation(dir);
            transform.rotation = Quaternion.Slerp(transform.rotation, rot, 10f * Time.deltaTime);
        }
    }

    void HandleBallHit()
    {
        if (hasHitBall) return;
        if (pawHitPoint == null || ballRb == null) return;

        // Distance uniquement en XZ (pour ignorer la hauteur)
        Vector3 pawXZ = new Vector3(pawHitPoint.position.x, 0f, pawHitPoint.position.z);
        Vector3 ballXZ = new Vector3(ballRb.position.x, 0f, ballRb.position.z);

        float d = Vector3.Distance(pawXZ, ballXZ);

        if (d <= hitDistance)
        {
            hasHitBall = true;
            Debug.Log("HIT TRIGGERED!");

            // Patte frappe la balle
            Vector3 dir = (ballRb.position - pawHitPoint.position);
            dir.y = 0f;

            if (dir.sqrMagnitude < 0.0001f)
                dir = transform.forward;
            else
                dir.Normalize();

            ballRb.AddForce(dir * hitForce, ForceMode.Impulse);
        }
    }
}