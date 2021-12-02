using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;


namespace OctopusController
{

    public class MyScorpionController
    {
        //TAIL
        Transform tailTarget;
        Transform tailEndEffector;
        MyTentacleController _tail;
        float animationRange;
        //LEGS
        Transform[] legTargets;
        Transform[] legFutureBases;
        Transform[] auxFutureBases;
        MyTentacleController[] _legs = new MyTentacleController[6];
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

            //Legs init
            for (int i = 0; i < LegRoots.Length; i++)
            {
                legMoving[i] = false;
                _legs[i] = new MyTentacleController();
                _legs[i].LoadTentacleJoints(LegRoots[i], TentacleMode.LEG);
                //TODO: initialize anything needed for the FABRIK implementation
                distances[i] = new float[_legs[i].Bones.Length - 1];
                copy[i] = new Vector3[_legs[i].Bones.Length];
                for (int j = 0; j < _legs[i].Bones.Length - 1; j++)
                    distances[i][j] = Vector3.Distance(_legs[i].Bones[j].position, _legs[i].Bones[j + 1].position);
            }
            animationRange = 0.5f;




        }

        public void InitTail(Transform TailBase)
        {
            _tail = new MyTentacleController();
            _tail.LoadTentacleJoints(TailBase, TentacleMode.TAIL);
            //TODO: Initialize anything needed for the Gradient Descent implementation
        }

        //TODO: Check when to start the animation towards target and implement Gradient Descent method to move the joints.
        public void NotifyTailTarget(Transform target)
        {

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
        //TODO: Implement the leg base animations and logic
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
                if(legMoving[i])
                { 
                    _legs[i].Bones[0].position = Vector3.Lerp(_legs[i].Bones[0].position, auxFutureBases[i].position, Time.deltaTime * 60);

                    if (Vector3.Distance(_legs[i].Bones[0].position, auxFutureBases[i].position) <= 0.04f)
                    {
                        legMoving[i] = false;
                    }

                }
            }
        }

        //TODO: implement Gradient Descent method to move tail if necessary
        private void updateTail()
        {

        }

        //TODO: implement fabrik method to move legs 
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

               //for (int j = 0; j < copy[i].Length - 1; j++)
               //    Debug.DrawLine(copy[i], copy[i + 1], Color.green, 0, false);
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