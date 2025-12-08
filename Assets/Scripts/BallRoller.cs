using UnityEngine;

public class BallRoller : MonoBehaviour
{
    [Tooltip("Direction to push the ball in world space.")]
    public Vector3 direction = Vector3.forward;
    public float forceAmount = 10f;

    void Start()
    {
        Rigidbody rb = GetComponent<Rigidbody>();
        Vector3 dir = direction.normalized;
        rb.AddForce(dir * forceAmount, ForceMode.Impulse);
    }
}