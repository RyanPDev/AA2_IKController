using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;


namespace OctopusController
{
    public delegate float ErrorFunction(Vector3 target, float[] solution);
    public class MyScorpionController
    {
        public struct PositionRotation
        {
            Vector3 position;
            Quaternion rotation;

            public PositionRotation(Vector3 position, Quaternion rotation)
            {
                this.position = position;
                this.rotation = rotation;
            }

            // PositionRotation to Vector3
            public static implicit operator Vector3(PositionRotation pr)
            {
                return pr.position;
            }
            // PositionRotation to Quaternion
            public static implicit operator Quaternion(PositionRotation pr)
            {
                return pr.rotation;
            }
        }
        //TAIL

        Transform tailTarget;
        Transform tailEndEffector;
        MyTentacleController _tail;
        Vector3[] Axis;
        float angle;
        float[] MinAngle;
        float[] MaxAngle;
        public ErrorFunction ErrorFunction;
        public float[] Solution = null;
        float[] t;
        public float DeltaGradient = 0.1f; // Used to simulate gradient (degrees)
        public float LearningRate = 10f; // How much we move depending on the gradient

        public float StopThreshold = 0.1f; // If closer than this, it stops
        public float SlowdownThreshold = 0.25f; // If closer than this, it linearly slows down

        // The offset at resting position
        Vector3[] StartOffset;

        // The initial one
        Vector3[] ZeroEuler;

        //LEGS
        Transform[] legTargets;
        Transform[] legFutureBases;
        Transform[] auxFutureBases;
        MyTentacleController[] _legs = new MyTentacleController[6];
        float animationRange;
        bool[] legMoving;

        ////Fabrik
        private Vector3[][] copy;
        private float[][] distances;
        //   private bool[] done;

        float threshHold = 0.03f;
        int maxIterations = 15;
        int iterations = 15;

        #region public

        public void InitLegs(Transform[] LegRoots, Transform[] LegFutureBases, Transform[] LegTargets)
        {
            _legs = new MyTentacleController[LegRoots.Length];
            legFutureBases = new Transform[LegFutureBases.Length];
            legTargets = new Transform[LegTargets.Length];
            auxFutureBases = new Transform[LegFutureBases.Length];
            legMoving = new bool[LegRoots.Length];
            legFutureBases = LegFutureBases;
            legTargets = LegTargets;
            distances = new float[LegRoots.Length][];
            copy = new Vector3[LegRoots.Length][];
            t = new float[LegRoots.Length];

            //Legs init
            for (int i = 0; i < LegRoots.Length; i++)
            {
                legMoving[i] = false;
                _legs[i] = new MyTentacleController();
                _legs[i].LoadTentacleJoints(LegRoots[i], TentacleMode.LEG);
                distances[i] = new float[_legs[i].Bones.Length - 1];
                t[i] = 0;
                copy[i] = new Vector3[_legs[i].Bones.Length];
                for (int j = 0; j < _legs[i].Bones.Length - 1; j++)
                    distances[i][j] = Vector3.Distance(_legs[i].Bones[j].position, _legs[i].Bones[j + 1].position);
            }
            animationRange = 0.5f;
        }

        public void InitTail(Transform TailBase)
        {
            ErrorFunction = DistanceFromTarget;
            _tail = new MyTentacleController();
            _tail.LoadTentacleJoints(TailBase, TentacleMode.TAIL);
            Solution = new float[_tail.Bones.Length];
            StartOffset = new Vector3[_tail.Bones.Length];
            Axis = new Vector3[_tail.Bones.Length];

            for (int i = 0; i < _tail.Bones.Length; i++)
            {
                StartOffset[i] = _tail.Bones[i].localPosition * 0.32622f;
                if (i >= 1)
                {
                    Axis[i] = new Vector3(1, 0, 0);
                    Solution[i] = _tail.Bones[i].localEulerAngles.x;
                   // Debug.Log(Solution[i]);
                }
                else
                {
                    Axis[i] = new Vector3(0, 0, 1);
                    Solution[i] = _tail.Bones[i].localEulerAngles.z;
                }
            }
        }

        //TODO: Check when to start the animation towards target and implement Gradient Descent method to move the joints.
        public void NotifyTailTarget(Transform target)
        {
            tailTarget = target;
            if (Vector3.Distance(tailTarget.position, _tail._endEffectorSphere.position) < 3)
            {
                if (ErrorFunction(tailTarget.position, Solution) > StopThreshold)
                {
                   // Debug.Log("Distance = " + Vector3.Distance(target.position, _tail._endEffectorSphere.position));
                    updateTail();
                }
            }
        }

        //TODO: Notifies the start of the walking animation
        public void NotifyStartWalk()
        {

        }

        //TODO: create the apropiate animations and update the IK from the legs and tail

        public void UpdateIK()
        {
            updateLegPos();
            updateLegs();
        }
        #endregion

        #region private
        private void updateLegPos()
        {
            //check for the distance to the futureBase, then if it's too far away start moving the leg towards the future base position
            //
            for (int i = 0; i < _legs.Length; i++)
            {
                if (Vector3.Distance(_legs[i].Bones[0].position, legFutureBases[i].position) > animationRange && !legMoving[i])
                {
                    legMoving[i] = true;
                    auxFutureBases[i] = legFutureBases[i];
                }
                if (legMoving[i])
                {
                    t[i] += Time.deltaTime * 10;
                    _legs[i].Bones[0].position = Vector3.Lerp(_legs[i].Bones[0].position, auxFutureBases[i].position, t[i]);

                    if (t[i] >= 1)
                    {
                        t[i] = 0;
                        legMoving[i] = false;
                    }
                }
            }
        }

        //TODO: implement Gradient Descent method to move tail if necessary
        private void updateTail()
        {
            for (int i = 0; i < _tail.Bones.Length; i++)
            {
                float gradient = CalculateGradient(tailTarget.position, Solution, i, DeltaGradient);
                Solution[i] -= LearningRate * gradient;

                if (Axis[i].x == 1) _tail.Bones[i].localEulerAngles = new Vector3(Solution[i], 0, 0);
                if (Axis[i].y == 1) _tail.Bones[i].localEulerAngles = new Vector3(0, Solution[i], 0);
                if (Axis[i].z == 1) _tail.Bones[i].localEulerAngles = new Vector3(0, 0, Solution[i]);
            }
        }

        public float CalculateGradient(Vector3 target, float[] _Solution, int i, float delta)
        {
            //TODO 
            float angle = _Solution[i];
            float f_x = DistanceFromTarget(target, _Solution);
            _Solution[i] += delta;
            float f_xFinal = DistanceFromTarget(target, _Solution);
            _Solution[i] = angle;

            return (f_xFinal - f_x) / delta;
        }

        // Returns the distance from the target, given a solution
        public float DistanceFromTarget(Vector3 target, float[] _Solution)
        {
            Vector3 point = ForwardKinematics(_Solution);
            return Vector3.Distance(point, target);
        }

        public PositionRotation ForwardKinematics(float[] _Solution)
        {
            Vector3 prevPoint = _tail.Bones[0].position;

            // Takes object initial rotation into account
            Quaternion rotation = new Quaternion(0, 0, 0, 1);

            //TODO
            for (int i = 1; i < _tail.Bones.Length; i++)
            {
                rotation *= Quaternion.AngleAxis(_Solution[i - 1], Axis[i - 1]);
                Vector3 aux = prevPoint + rotation * StartOffset[i];

                //Debug.DrawLine(aux, prevPoint);
                prevPoint = aux;
            }

            // The end of the effector
            return new PositionRotation(prevPoint, rotation);
        }

        private void updateLegs()
        {
            for (int i = 0; i < _legs.Length; i++)
            {
                for (int j = 0; j < _legs[i].Bones.Length; j++)
                {
                    copy[i][j] = _legs[i].Bones[j].position;
                }
            }

            for (int i = 0; i < _legs.Length; i++)
            {
                float targetRootDist = Vector3.Distance(copy[i][0], legTargets[i].position);
                Vector3 targetGoal = legTargets[i].position;

                // Update joint positions
                if (targetRootDist > distances[i].Sum())
                {
                    // The target is unreachable
                    targetGoal = copy[i][0] + (legTargets[i].position - copy[i][0]).normalized * distances[i].Sum();
                }
                iterations = maxIterations;


                // The target is reachable
                while (Vector3.Distance(copy[i][copy[i].Length - 1], targetGoal) > threshHold && iterations > 0)
                {
                    // STAGE 1: FORWARD REACHING
                    copy[i][copy[i].Length - 1] = targetGoal;

                    for (int j = copy[i].Length - 2; j >= 0; j--)
                        copy[i][j] = copy[i][j + 1] + ((copy[i][j] - copy[i][j + 1]).normalized * distances[i][j]);


                    // STAGE 2: BACKWARD REACHING
                    copy[i][0] = _legs[i].Bones[0].position;
                    for (int j = 1; j > copy[i].Length; j++)
                        copy[i][j] = copy[i][j - 1] + ((copy[i][j] - copy[i][j - 1]).normalized * distances[i][j]);


                    iterations--;
                }

                iterations = maxIterations;

                // Update original joint rotations

                for (int j = 0; j < _legs[i].Bones.Length - 1; j++)
                {
                    Vector3 currentDirection = _legs[i].Bones[j].TransformDirection(Vector3.up).normalized;
                    Vector3 endDirection = (copy[i][j + 1] - copy[i][j]).normalized;

                    if (Vector3.Dot(currentDirection, endDirection) > 1 - 0.01) continue; // está alineado

                    Vector3 axis = Vector3.Cross(currentDirection, endDirection).normalized;

                    float angle = Mathf.Acos((Vector3.Dot(currentDirection, endDirection))) * Mathf.Rad2Deg;
                    //angle *= Mathf.Rad2Deg;

                    _legs[i].Bones[j].Rotate(axis, angle, Space.World);
                }
            }

        }
        #endregion
    }
}