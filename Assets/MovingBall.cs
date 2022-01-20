using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MovingBall : MonoBehaviour
{
    [SerializeField]
    IK_tentacles _myOctopus;
    public GameObject SpherePrefab;
    //movement speed in units per second
    [Range(-1.0f, 1.0f)]
    [SerializeField]
    private float _movementSpeed = 5f;
    public bool shotInAction = false;
    Vector3 _dir;
    float angle, iSpeed;
    Vector3 initialPosition, endPosition, acceleration;
    const int MAX_ANGLE = 60;
    int counter = 0;
    float time, totalTime;
    public Vector3 initialVelocity;
    Vector3 unitDirection;
    Vector3 currentVelocity;
    Vector3 currentPosition;
    // Start is called before the first frame update
    int steps = 1000;
    float distance;

    Vector3 planarTarget;
    Vector3 planarPosition;
    float yOffset;
    public bool shotCalculated = false;
    List<GameObject> trajectoryList = new List<GameObject>();
    public List<Animator> spectators;
    public Transform target;
    void Start()
    {
        transform.rotation = Quaternion.identity;
        initialVelocity = new Vector3(0, 0, 0);
        acceleration = new Vector3(0, -9.8f, 0);
        time = 0;
        for (int i = 0; i < steps; i++)
        {
            GameObject p = Instantiate(SpherePrefab, new Vector3(0, 0, 0), Quaternion.identity);
            trajectoryList.Add(p);
        }
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        ////update the position
        if (shotInAction)
        {
            time += Time.fixedDeltaTime;
            currentPosition = initialPosition + (currentVelocity * time);
            transform.position = currentPosition;
            currentVelocity.y = initialVelocity.y + (acceleration.y * time);
            if (time > totalTime && counter % 2 == 0)
            {
                shotInAction = false;
            }
        }
        if (shotCalculated)
        {
            float auxTime = 0;
            Vector3 auxVelocity = initialVelocity;
            for (int i = 0; i < steps; i++)
            {
                auxTime += Time.fixedDeltaTime;
                trajectoryList[i].transform.position = initialPosition + (auxVelocity * auxTime);
                auxVelocity.y = initialVelocity.y + (acceleration.y * auxTime);
            }
        }

    }
    public void CalculateShot(float _power)
    {
        initialPosition = transform.position;
        currentPosition = initialPosition;

        endPosition = target.position;
        angle = (MAX_ANGLE * (1 - _power)) + 10; // Minim angle of 30
        angle *= Mathf.Deg2Rad;
        CalculateMovements();

        iSpeed = 1 / Mathf.Cos(angle) * Mathf.Sqrt((0.5f * -acceleration.y * Mathf.Pow(distance, 2)) / (distance * Mathf.Tan(angle) + yOffset));

        CalculateTime();
        initialVelocity = CalculateInitialVelocity();
        currentVelocity = initialVelocity;
        shotCalculated = true;



    }
    private Vector3 CalculateInitialVelocity()
    {
        Vector3 iVel = new Vector3(0, 0, 0);

        // Rotate our velocity to match the direction between the two objects
        Vector3 velocity = new Vector3(0, iSpeed * Mathf.Sin(angle) * 2, iSpeed * Mathf.Cos(angle));
        float angleBetweenObjects = Vector3.Angle(Vector3.forward, unitDirection) * (endPosition.x > transform.position.x ? 1 : -1);

        iVel = Quaternion.AngleAxis(angleBetweenObjects, Vector3.up) * velocity;
        // return iVel;
        return iVel;
    }
    private void CalculateTime()
    {
        totalTime = distance / (iSpeed * Mathf.Cos(angle));
    }
    public void ResetShot()
    {
        shotInAction = false;
        CallAnimations(false);
        shotCalculated = false;
        time = 0;

    }
    private void CalculateMovements()
    {
        // Positions of this object and the target on the same plane
        planarTarget = new Vector3(endPosition.x, 0, endPosition.z);
        planarPosition = new Vector3(transform.position.x, 0, transform.position.z);
        unitDirection = Vector3.Normalize(planarTarget - planarPosition);

        // Planar distance between objects
        distance = Vector3.Distance(planarTarget, planarPosition);

        // Distance along the y axis between objects
        yOffset = Mathf.Abs(endPosition.y - initialPosition.y);
    }
    public void CallAnimations(bool isShooting = true)
    {
        int _count = 0;
        foreach (Animator a in spectators)
        {
            a.SetBool("goal", isShooting);
            a.SetBool("InsideGoal", _count % 2 == 0 ? counter % 2 == 0 : counter % 2 != 0);
            _count++;
        }
    }
    private void OnCollisionEnter(Collision collision)
    {
        if (!shotInAction && collision.gameObject.tag == "Tail")
        {
            if (counter % 2 == 0)
                _myOctopus.NotifyShoot();

            counter++;
            CallAnimations(true);
            shotInAction = true;
        }

    }
    public Vector3 GetParabolaNextPosition(Vector3 position, Vector3 velocity, float gravity, float time)
    {
        return initialPosition + (velocity * time);
    }
}
