using UnityEngine;
 
// Script qui fait marcher le chat jusqu'à un point précis( walkpoint) et permet de pousser la balle avec sa patte
 
public class CatWalk : MonoBehaviour

{

    [Header("Marche du chat")]

    public Transform walkPoint;      // Le point où le chat doit s'arrêter 

    public float walkSpeed = 0.12f;  // Vitesse de déplacement du chat
 
    private Animator anim;           // Pour activer l'animation de marche

    private bool isWalking = true;   // Indique si le chat est en train de marcher

    private bool reached = false;    // Indique si le chat est arrivé au point de destination
 
    [Header("Contact avec la balle")]

    public Transform pawHitPoint;    // Point virtuel placé sur la patte 

    public Rigidbody ballRb;         // Rigidbody de la balle à pousser

    public float hitDistance = 0.07f;// Distance maximale pour détecter le contact

    public float hitForce = 2.5f;    // Force appliquée à la balle quand le chat la touche

    private bool hasHitBall = false; // bool pour empêche que la balle soit touché plusieurs fois
 
    void Start()

    {

        // On récupère l'Animator du chat dans format1 ( enfant du model)

        anim = GetComponentInChildren<Animator>();
 
        // Si un Animator existe, on lance l'animation de marche

        if (anim != null)

            anim.SetBool("isWalking", true); // Active le paramètre (isWalking) dans l'Animator

    }
 
    void Update()

    {

        // Avant de marcher, on vérifie si le chat doit frapper la balle

        HandleBallHit();
 
        // Si le chat est arrivé à destination ou s'il n'y a pas de point de marche le chat s'arrête

        if (reached || walkPoint == null) return;
 
        // Sinon, le chat continue de marcher vers le point de destination

        MoveToPoint();

    }
 
    void MoveToPoint()

    {

        // Position de la cible 

        Vector3 target = new Vector3(

            walkPoint.position.x,

            transform.position.y,

            walkPoint.position.z

        );
 
        // Distance entre le chat et le point cible

        float distance = Vector3.Distance(transform.position, target);
 
        // Si le chat est très proche du point 

        if (distance < 0.01f)

        {

            reached = true;     // On indique que le chat est arrivé

            isWalking = false;  // Il arrête sa marche
 
            // On place exactement le chat sur la position du point

            transform.position = target;
 
            // On met l'animation Idle donc il s'arrêt

            if (anim != null)

                anim.SetBool("isWalking", false);
 
            return;

        }
 
        //  Déplacement du chat vers la cible 

        float step = walkSpeed * Time.deltaTime; // Distance à parcourir cette frame

        transform.position = Vector3.MoveTowards(transform.position, target, step);
 
        // Rotation du chat pour qu'il regarde vers l'avant

        Vector3 dir = target - transform.position; // Direction vers la cible

        dir.y = 0f;  
 
        if (dir.sqrMagnitude > 0.0001f) // Si la direction n'est pas presque nulle

        {

            Quaternion rot = Quaternion.LookRotation(dir); // Rotation voulue

            transform.rotation = Quaternion.Slerp(transform.rotation, rot, 10f * Time.deltaTime);

            // Slerp permet une rotation douce et naturelle

        }

    }
 
    void HandleBallHit()

    {

        // Si le chat a déjà frappé la balle, on ne recommence pas on s'arret 

        if (hasHitBall) return;
 
        // Si pas de patte ou pas de balle a frappé 

        if (pawHitPoint == null || ballRb == null) return;
 
        // On compare uniquement X et Z  pour détecter un contact

        Vector3 pawXZ = new Vector3(pawHitPoint.position.x, 0f, pawHitPoint.position.z);

        Vector3 ballXZ = new Vector3(ballRb.position.x, 0f, ballRb.position.z);
 
        // Distance entre la patte et la balle

        float d = Vector3.Distance(pawXZ, ballXZ);
 
        // Si la balle est assez proche pour être touchée

        if (d <= hitDistance)

        {

            hasHitBall = true; // On évite de frapper plusieurs fois

            Debug.Log("HIT TRIGGERED!");
 
            // Direction de la force appliquée à la balle

            Vector3 dir = (ballRb.position - pawHitPoint.position);

            dir.y = 0f;
 
           

            if (dir.sqrMagnitude < 0.0001f)

                dir = transform.forward;

            else

                dir.Normalize(); // Normalisation pour une force constante
 
            // Application d une impulsion sur la balle

            ballRb.AddForce(dir * hitForce, ForceMode.Impulse);

        }

    }

}

 