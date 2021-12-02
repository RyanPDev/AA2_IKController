using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace OctopusController
{
    public enum TentacleMode { LEG, TAIL, TENTACLE };

    public class MyOctopusController
    {

        MyTentacleController[] _tentacles = new MyTentacleController[4];

        Transform _currentRegion;
        Transform _target;

        Transform[] _randomTargets;// = new Transform[4];


        float maxAngle = 10;

        float minAngle = 0;

        float _twistMin, _twistMax;
        float _swingMin, _swingMax;

        //todo: add here anything that you need

        private bool ScorpionHasShot = false;
        // Max number of tries before the system gives up (Maybe 10 is too high?)
        private int _mtries = 10;
        // The number of tries the system is at now
        private int _tries = 0;
        private float angle = 0f, distance = 0f;
        private Vector3 axis = Vector3.zero;
        private float[][] _theta;
        //private readonly float _epsilon = 0.1f;
        private Vector3[] tpos;

        #region public methods
        //DO NOT CHANGE THE PUBLIC METHODS!!

        public float TwistMin { set => _twistMin = value; }
        public float TwistMax { set => _twistMax = value; }
        public float SwingMin { set => _swingMin = value; }
        public float SwingMax { set => _swingMax = value; }

        public void TestLogging(string objectName)
        {
            Debug.Log("Hello, we are Marc and Ryan and we're initializing our Octopus Controller in object " + objectName);
        }

        public void Init(Transform[] tentacleRoots, Transform[] randomTargets)
        {
            _tentacles = new MyTentacleController[tentacleRoots.Length];
            tpos = new Vector3[randomTargets.Length];
            _theta = new float[tentacleRoots.Length][];

            _randomTargets = randomTargets;
            // foreach (Transform t in tentacleRoots)
            for (int i = 0; i < tentacleRoots.Length; i++)
            {
                _tentacles[i] = new MyTentacleController();
                _tentacles[i].LoadTentacleJoints(tentacleRoots[i], TentacleMode.TENTACLE);

                //TODO: initialize any variables needed in ccd
                _theta[i] = new float[_tentacles[i].Bones.Length];
                tpos[i] = _randomTargets[i].transform.position;

            }
           // Debug.LogError("ERROR");


            //TODO: use the regions however you need to make sure each tentacle stays in its region

            //////////PREGUNTAAAAAAA////////
        }

        public void NotifyTarget(Transform target, Transform region)
        {
            _currentRegion = region;
            _target = target;
        }

        public void NotifyShoot()
        {
            //TODO. what happens here?
            Debug.Log("Shoot");
        }

        public void UpdateTentacles()
        {
            //TODO: implement logic for the correct tentacle arm to stop the ball and implement CCD method

            update_ccd();
        }

        #endregion


        #region private and internal methods


        void update_ccd()
        {
            // if the target hasn't been reached
            if (!ScorpionHasShot)
            {
                // if the Max number of tries hasn't been reached
                if (_tries <= _mtries)
                {
                    // starting from the second last joint (the last being the end effector)
                    // going back up to the root
                    for (int i = 0; i < _tentacles.Length; i++)
                    {
                        for (int j = _tentacles[i].Bones.Length - 2; j >= 0; j--)
                        {
                            // The vector from the ith joint to the end effector
                            Vector3 r1 = _tentacles[i]._endEffectorSphere.transform.position - _tentacles[i].Bones[j].transform.position;

                            // The vector from the ith joint to the target
                            Vector3 r2 = tpos[i] - _tentacles[i].Bones[j].transform.position;

                            // to avoid dividing by tiny numbers
                            if (r1.magnitude * r2.magnitude <= 0.001f)
                            {
                                // cos ? sin? 
                                // TODO3
                            }
                            else
                            {
                                angle = Vector3.Angle(r1, r2);
                                axis = Vector3.Cross(r1, r2).normalized;
                            }

                            _theta[i][j] = SimpleAngle(angle);

                            // rotate the ith joint along the axis by theta degrees in the world space.
                            _tentacles[i].Bones[j].transform.Rotate(axis, _theta[i][j], Space.World);

                            // find the difference in the positions of the end effector and the target
                            distance = Vector3.Distance(tpos[i], _tentacles[i]._endEffectorSphere.transform.position);

                            // the target has moved, reset tries to 0 and change tpos
                            if (_randomTargets[i].transform.position != tpos[i])
                            {
                                _tries = 0;
                                tpos[i] = _randomTargets[i].transform.position;
                            }

                            GetSwing(_tentacles[i].Bones[j].transform.localRotation).ToAngleAxis(out float swingAngle, out Vector3 swingAxis);
                            swingAngle = Mathf.Clamp(swingAngle, minAngle, maxAngle);
                            _tentacles[i].Bones[j].localRotation = Quaternion.AngleAxis(swingAngle, swingAxis);
                        }
                    }

                    // increment tries
                    _tries++;
                }
            }

            #endregion
        }
        Quaternion GetTwist(Quaternion q)
        {
            return Quaternion.Normalize(new Quaternion(0, q.y, 0, q.w));
        }

        Quaternion GetSwing(Quaternion q)
        {
            return q * Quaternion.Inverse(GetTwist(q));
        }

        float SimpleAngle(float theta)
        {
            return Mathf.Clamp(theta, -180, 180);
        }
    }
}