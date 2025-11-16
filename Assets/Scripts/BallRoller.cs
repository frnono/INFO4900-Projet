using UnityEngine;

public class BallRoller : MonoBehaviour
{
    public float forceAmount = 10f;

    void Start()
    {
        Rigidbody rb = GetComponent<Rigidbody>();
        // Push along Z axis (forward)
        rb.AddForce(Vector3.forward * forceAmount, ForceMode.Impulse);
    }
}