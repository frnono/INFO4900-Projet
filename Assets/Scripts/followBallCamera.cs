using UnityEngine;

public class FollowCatBallCamera : MonoBehaviour
{
    [Header("Targets")]
    public Transform cat;
    public Transform ball;
    public Rigidbody ballRb;
    
    [Header("Camera Offsets")]
    private Vector3 currentOffset;          // actual offset used by camera
    public Vector3 catOffset = new Vector3(0f, 3f, -6f);
    public Vector3 ballOffset = new Vector3(0f, 3f, -6f);

    [Header("Room View (final shot)")]
    public Transform roomViewPoint;   // Drag your RoomViewPoint here
    public float roomSmooth = 2f;

    [Header("Smoothing")]
    public float followSmooth = 5f;
    public float rotationSmooth = 5f;   // 🔹 NEW: controls how fast the camera rotates

    [Header("Speed / timing")]
    public float movedSpeedThreshold = 0.05f;   // above this = ball is moving
    public float stopSpeedThreshold  = 0.03f;   // below this = almost stopped
    public float stopTimeRequired    = 1.0f;    // seconds below threshold before switching

    [Header("Look at targets")]
    public bool lookAtCat = true;
    public bool lookAtBall = true;

    private bool ballHasMoved = false;
    private float belowStopSpeedTimer = 0f;

    private enum FollowState
    {
        Cat,
        Ball,
        RoomView
    }

    [SerializeField]
    private FollowState state = FollowState.Cat;

    void Start()
    {
        currentOffset = catOffset;
    }

    void LateUpdate()
    {
        if (cat == null || ball == null || ballRb == null)
            return;

        float speed = ballRb.linearVelocity.magnitude;

        // 1) Has the ball ever started moving?
        if (!ballHasMoved && speed > movedSpeedThreshold)
        {
            ballHasMoved = true;
            state = FollowState.Ball;
        }

        // 2) If ball has moved, accumulate time while it stays slow
        if (ballHasMoved && speed < stopSpeedThreshold)
        {
            belowStopSpeedTimer += Time.deltaTime;
        }
        else
        {
            belowStopSpeedTimer = 0f;
        }

        // 3) If it's been slow long enough, switch to room view
        if (ballHasMoved && belowStopSpeedTimer >= stopTimeRequired && state != FollowState.RoomView)
        {
            state = FollowState.RoomView;
        }

        // --- STATES ---

        if (state == FollowState.Cat && !ballHasMoved)
        {
            // 🔵 FOLLOW CAT
            currentOffset = Vector3.Lerp(currentOffset, catOffset, followSmooth * Time.deltaTime);

            Vector3 desiredPos = cat.position + currentOffset;
            transform.position = Vector3.Lerp(transform.position, desiredPos, followSmooth * Time.deltaTime);

            if (lookAtCat)
                SmoothLookAt(cat.position);
        }
        else if (state == FollowState.Ball)
        {
            // 🟢 FOLLOW BALL
            currentOffset = Vector3.Lerp(currentOffset, ballOffset, followSmooth * Time.deltaTime);

            Vector3 desiredPos = ball.position + currentOffset;
            transform.position = Vector3.Lerp(transform.position, desiredPos, followSmooth * Time.deltaTime);

            if (lookAtBall)
                SmoothLookAt(ball.position);
        }
        else if (state == FollowState.RoomView)
        {
            // 🔴 FINAL ROOM VIEW
            if (roomViewPoint != null)
            {
                transform.position = Vector3.Lerp(transform.position, roomViewPoint.position, roomSmooth * Time.deltaTime);
                transform.rotation = Quaternion.Lerp(transform.rotation, roomViewPoint.rotation, roomSmooth * Time.deltaTime);
            }
        }
    }

    // 🔹 NEW: smooth rotation instead of instant LookAt
    private void SmoothLookAt(Vector3 targetPos)
    {
        Vector3 direction = targetPos - transform.position;
        if (direction.sqrMagnitude < 0.0001f) return; // avoid NaN if super close

        Quaternion targetRot = Quaternion.LookRotation(direction.normalized);
        transform.rotation = Quaternion.Lerp(
            transform.rotation,
            targetRot,
            rotationSmooth * Time.deltaTime
        );
    }
}
