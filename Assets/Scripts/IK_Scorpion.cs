using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OctopusController;
using UnityEngine.UI;

public class IK_Scorpion : MonoBehaviour
{
    public IK_tentacles _myOctopus;
    MyScorpionController _myController = new MyScorpionController();

    [Header("Body")]
    float animTime;
    public float animDuration = 5;
    bool animPlaying = false;
    public Transform Body;
    public Transform StartPos;
    public Transform EndPos;

   // public Transform target;
    public GameObject ball;
    MovingBall movingBall;
    public Slider strengthSlider;

    [Header("Tail")]
    public Transform tailTarget;
    public Transform tail;

    [Header("Legs")]
    public Transform[] legs;
    public Transform[] legTargets;
    public Transform[] futureLegBases;
    Vector3 position;
    enum AnimState
    {
        CALCULATING_FORCE, CALCULATING_EFFECT, MOVING
    }

    AnimState currentState = AnimState.CALCULATING_FORCE;

    float time = 0;
    float startTime = 0;
    [SerializeField] private const float MAX_FORCE = 100;
    float radius;
    void Start()
    {
        _myController.InitLegs(legs, futureLegBases, legTargets);
        _myController.InitTail(tail);
        _myController.InitBody(Body);
        movingBall = ball.GetComponent<MovingBall>();
        position = ball.transform.position;
        radius = ball.GetComponent<SphereCollider>().radius * ball.transform.localScale.x;
    }

    void Update()
    {
        if (animPlaying)
            animTime += Time.deltaTime;

        NotifyTailTarget();

        time = Time.time;

        if (animTime < animDuration)
        {
            Body.position = Vector3.Lerp(StartPos.position, EndPos.position, animTime / animDuration);
           
        }
        else if (animTime >= animDuration && animPlaying)
        {
            Body.position = EndPos.position;
            animPlaying = false;
        }

        _myController.UpdateIK();
        CheckInput();
    }
    

    //Function to send the tail target transform to the dll
    public void NotifyTailTarget()
    {
        tailTarget.position = ball.transform.position- Vector3.Normalize(movingBall.initialVelocity) * radius;
        _myController.NotifyTailTarget(tailTarget);
    }

    //Trigger Function to start the walk animation
    public void NotifyStartWalk()
    {
        _myController.NotifyStartWalk();
        movingBall.CalculateShot(strengthSlider.value);
    }

    private void CheckInput()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            startTime = time;
            if (currentState == AnimState.MOVING)
            {
                Body.rotation = Quaternion.identity;
                Body.localRotation = Quaternion.identity;
                Body.localPosition = Vector3.zero;
                Body.position = StartPos.position;
                animTime = 0;
                animPlaying = false;
                NotifyStartWalk();
            }
        }

        if (Input.GetKey(KeyCode.Space) && currentState == AnimState.CALCULATING_FORCE)
        {
            strengthSlider.value = Mathf.PingPong(time - startTime, 1);
        }

        if (Input.GetKeyUp(KeyCode.Space))
        {
            if (currentState == AnimState.CALCULATING_FORCE)
            {                
                NotifyStartWalk();
                animTime = 0;
                animPlaying = true;
                currentState = AnimState.MOVING;
            }
            else if (currentState == AnimState.MOVING)
            {
                currentState = AnimState.CALCULATING_FORCE;
                _myController.ResetScopion();
                ball.transform.position = position;
                
                movingBall.ResetShot();
               
            }
        }
    }

 
}
