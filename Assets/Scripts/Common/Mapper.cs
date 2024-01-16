using UnityEngine;
using Microsoft.Azure.Kinect.BodyTracking;

public class Mapper : MonoBehaviour
{
    public static HumanBodyBones ToHumanBodyBonesFrom(JointId joint)
    {
        // https://docs.microsoft.com/en-us/azure/Kinect-dk/body-joints
        return joint switch
        {
            JointId.Pelvis => HumanBodyBones.Hips,
            JointId.SpineNavel => HumanBodyBones.Spine,
            JointId.SpineChest => HumanBodyBones.Chest,
            // JointId.Neck => HumanBodyBones.Neck,
            JointId.Head => HumanBodyBones.Head,
            JointId.HipLeft => HumanBodyBones.LeftUpperLeg,
            JointId.KneeLeft => HumanBodyBones.LeftLowerLeg,
            JointId.AnkleLeft => HumanBodyBones.LeftFoot,
            // JointId.FootLeft => HumanBodyBones.LeftToes,
            JointId.HipRight => HumanBodyBones.RightUpperLeg,
            JointId.KneeRight => HumanBodyBones.RightLowerLeg,
            JointId.AnkleRight => HumanBodyBones.RightFoot,
            // JointId.FootRight => HumanBodyBones.RightToes,
            // JointId.ClavicleLeft => HumanBodyBones.LeftShoulder,
            JointId.ShoulderLeft => HumanBodyBones.LeftUpperArm,
            JointId.ElbowLeft => HumanBodyBones.LeftLowerArm,
            JointId.WristLeft => HumanBodyBones.LeftHand,
            // JointId.ClavicleRight => HumanBodyBones.RightShoulder,
            JointId.ShoulderRight => HumanBodyBones.RightUpperArm,
            JointId.ElbowRight => HumanBodyBones.RightLowerArm,
            JointId.WristRight => HumanBodyBones.RightHand,
            _ => HumanBodyBones.LastBone
        };
    }
}
