using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MovingBall : MonoBehaviour
{
    [SerializeField]
    IK_tentacles _myOctopus;

    //movement speed in units per second
    [Range(-1.0f, 1.0f)]
    [SerializeField]
    private float _movementSpeed = 5f;
    bool shotInAction = false;
    Vector3 _dir;
    float angle,acceleration,iSpeed;
    Vector3 initialPosition,endPosition;
    const int MAX_ANGLE = 80;
    int counter = 0;
    float verticalMovement, horizontalMovement;
    float time,totalTime;
    Vector3 initialVelocity;
    Vector3 unitDirection;
    Vector3 currentVelocity;
    Vector3 currentPosition;
    // Start is called before the first frame update
    float distance;
    Vector3 planarTarget;
     Vector3 planarPosition;
    float yOffset;
    void Start()
    {
        transform.rotation = Quaternion.identity;
        acceleration = -9.81f;
        time = 0;
    }

    // Update is called once per frame
    void Update()
    {
        ////get the Input from Horizontal axis
        //float horizontalInput = Input.GetAxis("Horizontal");
        ////get the Input from Vertical axis
        //float verticalInput = Input.GetAxis("Vertical");

        ////update the position
        if(shotInAction)
        {
            currentPosition = GetParabolaNextPosition(transform.position, currentVelocity, acceleration, time);
            transform.position =  currentPosition;
            time += Time.deltaTime;
            currentVelocity.y += (acceleration) * Time.deltaTime;
            if(time > totalTime)
            {
               // shotInAction = false;
            }
        }
        


        //transform.position = transform.position + new Vector3(-horizontalInput * _movementSpeed * Time.deltaTime, verticalInput * _movementSpeed * Time.deltaTime, 0);

    }
    public void CalculateShot(float _power, Vector3 _endPosition)
    {
        initialPosition = transform.position;
        currentPosition = initialPosition;

        endPosition = _endPosition;
        angle = (MAX_ANGLE * _power) + 5; // Minim angle of 5
        //angle *= Mathf.Deg2Rad;
        CalculateMovements();
        //float omega = Mathf.Sqrt(Mathf.Pow(-acceleration * 0.5f, 2) * Mathf.Pow(horizontalMovement, 2) / (verticalMovement- horizontalMovement));
        // iSpeed = omega / Mathf.Cos(angle);
        iSpeed = (1 / Mathf.Cos(angle)) * Mathf.Sqrt((0.5f * acceleration * Mathf.Pow(distance, 2)) / (distance * Mathf.Tan(angle) + yOffset));

        CalculateTime(); 
        initialVelocity = CalculateInitialVelocity();
        currentVelocity = initialVelocity;
  
    }
    private Vector3 CalculateInitialVelocity()
    {
        Vector3 iVel = new Vector3(0,0,0);

       //iVel.y = iSpeed * Mathf.Sin(angle);
       //
       //
       //iVel.x = iSpeed * Mathf.Cos(angle) * unitDirection.x;
       //iVel.z = iSpeed * Mathf.Cos(angle) * unitDirection.z;
        Vector3 velocity = new Vector3(0, iSpeed * Mathf.Sin(angle), iSpeed * Mathf.Cos(angle));

        float angleBetweenObjects = Vector3.Angle(Vector3.forward, planarTarget - planarPosition) * (endPosition.x > transform.position.x ? 1 : -1); 
        iVel = Quaternion.AngleAxis(angleBetweenObjects, Vector3.up) * velocity;
        // return iVel;
        return iVel;
    }
    private void CalculateTime()
    {
        totalTime = horizontalMovement / (iSpeed * Mathf.Cos(angle));
    }
    private void CalculateMovements()
    {
        // Positions of this object and the target on the same plane
        planarTarget    = new Vector3(endPosition.x, 0, endPosition.z);
        planarPosition = new Vector3(transform.position.x, 0, transform.position.z);

        // Planar distance between objects
        distance = Vector3.Distance(planarTarget, planarPosition);
        // Distance along the y axis between objects
        yOffset = initialPosition.y - endPosition.y;
        

    }
    private void OnCollisionEnter(Collision collision)
    {
        if (!shotInAction)
        {
            if (counter % 2 == 0)
                _myOctopus.NotifyShoot();

            counter++;

            shotInAction = true;
        }

    }
    public static Vector3 GetParabolaNextPosition(Vector3 position, Vector3 velocity, float gravity, float time)
    {
        velocity.y += (-gravity) * time;
        return position + velocity * time;
    }
}
