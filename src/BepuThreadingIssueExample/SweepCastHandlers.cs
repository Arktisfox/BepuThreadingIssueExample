using BepuPhysics.Collidables;
using BepuPhysics;
using System.Numerics;
using BepuPhysics.Trees;
using System.Runtime.CompilerServices;

namespace BepuThreadingIssueExample
{
    unsafe struct DemoRayHandler : IRayHitHandler
    {
        public bool Hit;
        public Vector3 HitPos;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowTest(CollidableReference collidable)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowTest(CollidableReference collidable, int childIndex)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void OnRayHit(in RayData ray, ref float maximumT, float t, in Vector3 normal, CollidableReference collidable, int childIndex)
        {
            Hit = true;
            HitPos = ray.Origin + (ray.Direction * t);
        }
    }

    public unsafe struct DemoSweepHandler : ISweepHitHandler
    {
        public int HitCount;

        public bool AllowTest(CollidableReference collidable)
        {
            return true;
        }

        public bool AllowTest(CollidableReference collidable, int child)
        {
            return true;
        }

        public void OnHit(ref float maximumT, float t, in Vector3 hitLocation, in Vector3 hitNormal, CollidableReference collidable)
        {
            HitCount++;
        }

        public void OnHitAtZeroT(ref float maximumT, CollidableReference collidable)
        {

            HitCount++;
        }
    }
}
