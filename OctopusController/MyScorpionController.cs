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
        MyTentacleController[] _legs = new MyTentacleController[6];
        bool[] legMoving;

        ////Fabrik
        private Vector3[][] copy;
        private float[][] distances;
        private bool[] done;

        float threshHold = 0.03f;
        int maxIterations = 15;
        int iterations = 15;

        #region public
        public void InitLegs(Transform[] LegRoots, Transform[] LegFutureBases, Transform[] LegTargets)
        {
            _legs = new MyTentacleController[LegRoots.Length];
            legFutureBases = new Transform[LegFutureBases.Length];
            legTargets = new Transform[LegTargets.Length];
            legMoving = new bool[LegRoots.Length];
            legFutureBases = LegFutureBases;
            legTargets = LegTargets;

            //Legs init
            for (int i = 0; i < LegRoots.Length; i++)
            {
                legMoving[i] = false;
                _legs[i] = new MyTentacleController();
                _legs[i].LoadTentacleJoints(LegRoots[i], TentacleMode.LEG);
                //TODO: initialize anything needed for the FABRIK implementation
            }
            animationRange = Vector3.Distance(_legs[0].Bones[0].position, legFutureBases[0].position);

           // distances = new float[LegRoots.Length];
           // copy = new Vector3[joints.Length];
           //
           // for (int i = 0; i < joints.Length - 1; i++)
           // {
           //     distances[i] = Vector3.Distance(joints[i].position, joints[i + 1].position);
           // }


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
                if (Vector3.Distance(_legs[i].Bones[0].position, legFutureBases[i].position) > animationRange || legMoving[i])
                {
                    legMoving[i] = true;
                    _legs[i].Bones[0].position = Vector3.Lerp(_legs[i].Bones[0].position, legFutureBases[i].position, Time.deltaTime * 80);
                   
                    if (Vector3.Distance(_legs[i].Bones[0].position, legFutureBases[i].position) <= 0.04f)
                    {
                        legMoving[i] = false;
                    }
                    //_legs[i].Bones[0].position = legFutureBases[i].position;
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

        }
        #endregion
    }
}
