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
        int tentacleToMove;
        Transform[] _randomTargets;
        float t = 0, tl = 0;
        float maxAngle = 5.3f;
        float minAngle = 0;

        float _twistMin, _twistMax, timer, maxTime;
        float _swingMin, _swingMax;

        //todo: add here anything that you need

        private bool ScorpionHasShot = false, LerpingBack = false;
        // Max number of tries before the system gives up (Maybe 10 is too high?)

        private int _mtries = 10;

        // The number of tries the system is at now
        private int _tries = 0;
        private float angle = 0f, distance = 0f;
        private Vector3 axis = Vector3.zero;
        Vector3 _auxTarget, _auxTarget2;
        private float[][] _theta;
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
            maxTime = 3;
            timer = maxTime;
            for (int i = 0; i < tentacleRoots.Length; i++)
            {
                _tentacles[i] = new MyTentacleController();
                _tentacles[i].LoadTentacleJoints(tentacleRoots[i], TentacleMode.TENTACLE);

                _theta[i] = new float[_tentacles[i].Bones.Length];
                tpos[i] = _randomTargets[i].transform.position;
            }
        }

        public void NotifyTarget(Transform target, Transform region)
        {
            if (region.childCount == 1)
            {
                _currentRegion = region;
                _target = target;

                for (int i = 0; i < _tentacles.Length; i++)
                {
                    if (region.GetChild(0) == _randomTargets[i])
                    {
                        tentacleToMove = i;
                        _auxTarget2 = _randomTargets[tentacleToMove].position;
                    }
                }
            }

        }

        public void NotifyShoot()
        {
            ScorpionHasShot = true;
            Debug.Log("Shoot");
        }

        public void UpdateTentacles()
        {
            update_ccd();
        }

        #endregion

        #region private and internal methods

        void update_ccd()
        {
            if (ScorpionHasShot)
            {
                timer -= Time.deltaTime;
                t += Time.deltaTime * 3;
                _auxTarget = Vector3.Lerp(_auxTarget2, _target.position, t);
                if (timer <= 0)
                {
                    t = 0;
                    tl = 0;
                    LerpingBack = true;
                    ScorpionHasShot = false;
                    timer = maxTime;
                }
            }
            if (LerpingBack)
                tl += Time.deltaTime * 3;

            // if the Max number of tries hasn't been reached
            if (_tries <= _mtries)
            {
                for (int i = 0; i < _tentacles.Length; i++)
                {
                    // starting from the second last joint (the last being the end effector)
                    // going back up to the root
                    for (int j = _tentacles[i].Bones.Length - 2; j >= 0; j--)
                    {
                        // The vector from the ith joint to the end effector
                        Vector3 r1 = _tentacles[i]._endEffectorSphere.transform.position - _tentacles[i].Bones[j].transform.position;

                        // The vector from the ith joint to the target
                        Vector3 r2 = tpos[i] - _tentacles[i].Bones[j].transform.position;

                        // to avoid dividing by tiny numbers
                        if (r1.magnitude * r2.magnitude >= 0.001f)
                        {
                            angle = Vector3.Angle(r1, r2);
                            axis = Vector3.Cross(r1, r2).normalized;
                        }

                        _theta[i][j] = SimpleAngle(angle);

                        // rotate the ith joint along the axis by theta degrees in the world space.
                        _tentacles[i].Bones[j].transform.Rotate(axis, _theta[i][j], Space.World);

                        // find the difference in the positions of the end effector and the target
                        distance = Vector3.Distance(tpos[i], _tentacles[i]._endEffectorSphere.transform.position);

                        if (ScorpionHasShot && tentacleToMove == i)
                        {
                            // the target has moved, reset tries to 0 and change tpos
                            if (_auxTarget != tpos[i])
                            {
                                _tries = 0;
                                tpos[i] = _auxTarget;
                            }
                        }
                        else
                        {
                            // the target has moved, reset tries to 0 and change tpos
                            if (_randomTargets[i].position != tpos[i])
                            {
                                _tries = 0;
                                if (LerpingBack && tentacleToMove == i)
                                {
                                    tpos[i] = Vector3.Lerp(_auxTarget, _randomTargets[i].position, tl);
                                    if (tl >= 1)
                                        LerpingBack = false;
                                }
                                else
                                    tpos[i] = _randomTargets[i].position;
                            }
                        }

                        GetSwing(_tentacles[i].Bones[j].localRotation).ToAngleAxis(out float swingAngle, out Vector3 swingAxis);
                        swingAngle = Mathf.Clamp(swingAngle, minAngle, maxAngle);
                        _tentacles[i].Bones[j].localRotation = Quaternion.AngleAxis(swingAngle, swingAxis);

                        //Codi per intentar fixar la rotacio dels joints en un sol pla, no acaba de funcionar del tot bé

                        //Quaternion SwingQuat = GetSwing(_tentacles[i].Bones[j].localRotation);
                        //SwingQuat.ToAngleAxis(out float swingAngle, out Vector3 swingAxis);
                        //swingAngle = Mathf.Clamp(swingAngle, minAngle, maxAngle);
                        //
                        //SwingQuat = Quaternion.AngleAxis(swingAngle, swingAxis);
                        //
                        //_tentacles[i].Bones[j].localRotation = new Quaternion(SwingQuat.x, 0, 0, SwingQuat.w);
            }
                }

                // increment tries
                _tries++;
            }

            #endregion
        }

        //Quaternion GetCameraRotation(Quaternion q)
        //{
        //    return Quaternion.Normalize(new Quaternion(0, 0, 1, 1) * Quaternion.Inverse(new Quaternion(q.x, 0, 0, q.w)));
        //}

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