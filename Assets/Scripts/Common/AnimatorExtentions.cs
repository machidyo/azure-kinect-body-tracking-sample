using System.Linq;
using UnityEngine;

public static class AnimatorExtentions
{
    public static SkeletonBone GetSkeletonBone(this Animator animator, string boneName)
    {
        return animator.avatar.humanDescription.skeleton
            .FirstOrDefault(sb => sb.name == boneName || sb.name == $"{boneName}(Clone)");
    }
}
