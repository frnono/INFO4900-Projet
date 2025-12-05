using UnityEngine;

public class CatWalk : MonoBehaviour
{
    [Header("Point d'arrêt")]
    public Transform walkPoint;      // Destination unique
    public float walkSpeed = 1.2f;   // Vitesse du chat
    public float stopDistance = 0.2f;

    private Animator anim;
    private bool walking = true;
    private bool finished = false;

    void Start()
    {
        anim = GetComponentInChildren<Animator>();

        if (anim != null)
            anim.SetBool("isWalking", true);
    }

    void Update()
    {
        if (finished || !walking) return;

        WalkToPoint();
    }

    // --------- MARCHE NORMALE ---------
    void WalkToPoint()
    {
        if (walkPoint == null) return;

        Vector3 dir = walkPoint.position - transform.position;
        dir.y = 0f;

        // Si assez proche ? s'arrêter
        if (dir.magnitude < stopDistance)
        {
            walking = false;
            finished = true;

            if (anim != null)
                anim.SetBool("isWalking", false); // Passe en Idle

            return;
        }

        // Avancer
        transform.position += dir.normalized * walkSpeed * Time.deltaTime;

        // Rotation douce vers la destination
        if (dir.sqrMagnitude > 0.001f)
        {
            Quaternion look = Quaternion.LookRotation(dir);
            transform.rotation = Quaternion.Slerp(transform.rotation, look, 10f * Time.deltaTime);
        }
    }
}
