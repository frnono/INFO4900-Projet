using UnityEngine;

public class FollowBallCamera : MonoBehaviour
{
    [Header("Ball target")]
    public Transform target;       
    public Rigidbody targetRb;     // Assign the ball's Rigidbody

    [Header("Follow settings")]
    public Vector3 followOffset = new Vector3(0f, 3f, -6f);
    public float followSmooth = 5f;

    [Header("Room overview settings")]
    public Transform roomViewPoint;   // Empty object where you want the camera to go
    public float roomSmooth = 2f;

    [Header("Speed thresholds")]
    public float stopSpeedThreshold = 0.1f;   // Below this = "stopped"
    public float movedSpeedThreshold = 0.2f;  // Above this at least once = "ball has moved"

    [Header("Look at target")]
    public bool lookAtBall = true;

    // Internal state
    private bool ballHasMoved = false;

    void LateUpdate()
    {
        if (target == null || targetRb == null)
            return;

        float speed = targetRb.linearVelocity.magnitude;

        // If the ball ever goes above "moved" threshold, remember it
        if (speed > movedSpeedThreshold)
        {
            ballHasMoved = true;
        }

        bool ballStopped = ballHasMoved && speed < stopSpeedThreshold;

        if (!ballStopped)
        {
            // FOLLOW BALL MODE (before it ever moves, and while moving)
            Vector3 desiredPos = target.position + followOffset;
            transform.position = Vector3.Lerp(transform.position, desiredPos, followSmooth * Time.deltaTime);

            if (lookAtBall)
                transform.LookAt(target);
        }
        else
        {
            // ROOM VIEW MODE (only after: moved at least once -> then stopped)
            if (roomViewPoint != null)
            {
                transform.position = Vector3.Lerp(transform.position, roomViewPoint.position, roomSmooth * Time.deltaTime);
                transform.rotation = Quaternion.Lerp(transform.rotation, roomViewPoint.rotation, roomSmooth * Time.deltaTime);
            }
        }
    }
}
