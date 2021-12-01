using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace OctopusController
{
    internal class MyTentacleController

    //MAINTAIN THIS CLASS AS INTERNAL
    {
        TentacleMode tentacleMode;
        Transform[] _bones;
        public Transform _endEffectorSphere;
        List<Transform> list = new List<Transform>();
        Transform child;
        public Transform[] Bones { get => _bones; }

        //Exercise 1.
        public Transform[] LoadTentacleJoints(Transform root, TentacleMode mode)
        {
            //TODO: add here whatever is needed to find the bones forming the tentacle for all modes
            //you may want to use a list, and then convert it to an array and save it into _bones

            tentacleMode = mode;

            switch (tentacleMode)
            {
                case TentacleMode.LEG:
                    //TODO: in _endEffectorsphere you keep a reference to the base of the leg
                    child = root.GetChild(0);
                        list.Add(child);

                    while (child.childCount == 2)
                    {
                        child = child.GetChild(1);
                        list.Add(child);
                    }
                    _bones = list.ToArray<Transform>();
                    _endEffectorSphere = _bones[_bones.Length - 1];
                    break;
                case TentacleMode.TAIL:
                    //TODO: in _endEffectorsphere you keep a reference to the red sphere
                   // _bones = root.GetComponentsInChildren<Transform>();
                    child = root.GetComponent<Transform>();

                    while (child.childCount == 2)
                    {
                        list.Add(child);
                        child = child.GetChild(1);
                    }
                    _bones = list.ToArray<Transform>();
                    _endEffectorSphere = _bones[_bones.Length - 1];
                    break;
                case TentacleMode.TENTACLE:
                    //TODO: in _endEffectorsphere you keep a reference to the sphere with a collider attached to the endEffector
                    //_bones = root.GetComponentsInChildren<Transform>();
                    child = root.GetChild(0).GetChild(0);
                    while (child.childCount == 1)
                    {
                        list.Add(child);
                        child = child.GetChild(0);
                    }
                    _bones = list.ToArray<Transform>();
                    _endEffectorSphere = _bones[_bones.Length - 1];
                    break;
            }
            return Bones;
        }
    }
}
