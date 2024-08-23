using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities.Memory;
using System.Numerics;

namespace BepuThreadingIssueExample
{
    internal class Program
    {
        static BufferPool BufferPool;
        static Simulation Simulation;

        static TypedIndex CreateTriangleMesh(Vector3 offset)
        {
            BufferPool.Take<Triangle>(2, out var triBuffer);

            ref var triA = ref triBuffer[0];
            triA.A = new Vector3(-1, 0, -1) + offset;
            triA.B = new Vector3(1, 0, -1) + offset;
            triA.C = new Vector3(1, 0, 1) + offset;

            ref var triB = ref triBuffer[0];
            triA.A = new Vector3(-1, 0, -1) + offset;
            triA.B = new Vector3(1, 0, 1) + offset;
            triA.C = new Vector3(-1, 0, 1) + offset;

            var mesh = new Mesh(triBuffer, Vector3.One, BufferPool);
            var shapeIndex = Simulation.Shapes.Add(mesh);
            return shapeIndex;
        }

        static void Main(string[] args)
        {
            BufferPool = new BufferPool();
            Simulation = Simulation.Create(BufferPool, new NoCollisionCallbacks(), new EmptyPoseIntegratorCallbacks(), new SolveDescription(8, 1));

            var msh1 = CreateTriangleMesh(Vector3.Zero);
            var msh2 = CreateTriangleMesh(Vector3.UnitZ);

            BufferPool.Take<CompoundChild>(2, out var children);
            children[0].LocalPosition = Vector3.Zero;
            children[0].LocalOrientation = Quaternion.Identity;
            children[0].ShapeIndex = msh1;

            children[1].LocalPosition = Vector3.Zero;
            children[1].LocalOrientation = Quaternion.Identity;
            children[1].ShapeIndex = msh2;

            //***************************
            //*  COMPOUND SWEEP FAILURE *
            //***************************
            var compoundShape = Simulation.Shapes.Add(new Compound(children));
            var compoundHandle = Simulation.Statics.Add(new StaticDescription(RigidPose.Identity, compoundShape));
            
            try
            {
                var sweeper = new Sphere(5.0f);
                var sweepHandler = new DemoSweepHandler();
                Simulation.Sweep(sweeper, RigidPose.Identity, new BodyVelocity(Vector3.Zero), float.MaxValue, BufferPool, ref sweepHandler);  // <Error source
            }
            catch (Exception ex)
            {
                /*
                   System.InvalidOperationException: Nonconvex shapes are not required to have a maximum radius or angular expansion implementation. This should only ever be called on convexes.
                   at BepuPhysics.Collidables.ShapeBatch.ComputeBounds(Int32 shapeIndex, Quaternion orientation, Single& maximumRadius, Single& maximumAngularExpansion, Vector3& min, Vector3& max)
                   at BepuPhysics.Collidables.Compound.FindLocalOverlaps[TOverlaps](Vector3 min, Vector3 max, Vector3 sweep, Single maximumT, BufferPool pool, Shapes shapes, Void* overlapsPointer)
                   at BepuPhysics.CollisionDetection.SweepTasks.ConvexCompoundSweepOverlapFinder`2.FindOverlaps(TShapeA& shapeA, Quaternion orientationA, BodyVelocity& velocityA, TCompoundB& compoundB, Vector3 offsetB, Quaternion orientationB, BodyVelocity& velocityB, Single maximumT, Shapes shapes, BufferPool pool, ChildOverlapsCollection& overlaps)
                   at BepuPhysics.CollisionDetection.SweepTasks.ConvexCompoundSweepOverlapFinder`2.BepuPhysics.CollisionDetection.SweepTasks.IConvexCompoundSweepOverlapFinder<TShapeA,TCompoundB>.FindOverlaps(TShapeA& shapeA, Quaternion orientationA, BodyVelocity& velocityA, TCompoundB& compoundB, Vector3 offsetB, Quaternion orientationB, BodyVelocity& velocityB, Single maximumT, Shapes shapes, BufferPool pool, ChildOverlapsCollection& overlaps)
                   at BepuPhysics.CollisionDetection.SweepTasks.ConvexCompoundSweepTask`4.PreorderedTypeSweep[TSweepFilter](Void* shapeDataA, Quaternion orientationA, BodyVelocity& velocityA, Void* shapeDataB, Vector3 offsetB, Quaternion orientationB, BodyVelocity& velocityB, Single maximumT, Single minimumProgression, Single convergenceThreshold, Int32 maximumIterationCount, Boolean flipRequired, TSweepFilter& filter, Shapes shapes, SweepTaskRegistry sweepTasks, BufferPool pool, Single& t0, Single& t1, Vector3& hitLocation, Vector3& hitNormal)
                   at BepuPhysics.CollisionDetection.SweepTask.Sweep[TSweepFilter](Void* shapeDataA, Int32 shapeTypeA, Quaternion orientationA, BodyVelocity& velocityA, Void* shapeDataB, Int32 shapeTypeB, Vector3 offsetB, Quaternion orientationB, BodyVelocity& velocityB, Single maximumT, Single minimumProgression, Single convergenceThreshold, Int32 maximumIterationCount, TSweepFilter& filter, Shapes shapes, SweepTaskRegistry sweepTasks, BufferPool pool, Single& t0, Single& t1, Vector3& hitLocation, Vector3& hitNormal)
                   at BepuPhysics.Trees.Tree.Sweep[TLeafTester](Vector3 expansion, Vector3 origin, Vector3 direction, TreeRay* treeRay, TLeafTester& sweepTester)
                   at BepuPhysics.CollisionDetection.BroadPhase.Sweep[TSweepTester](Vector3 min, Vector3 max, Vector3 direction, Single maximumT, TSweepTester& sweepTester)
                   at BepuPhysics.Simulation.Sweep[TShape,TSweepHitHandler](TShape shape, RigidPose& pose, BodyVelocity& velocity, Single maximumT, BufferPool pool, TSweepHitHandler& hitHandler, Single minimumProgression, Single convergenceThreshold, Int32 maximumIterationCount)
                   at BepuPhysics.Simulation.Sweep[TShape,TSweepHitHandler](TShape& shape, RigidPose& pose, BodyVelocity& velocity, Single maximumT, BufferPool pool, TSweepHitHandler& hitHandler)
                   at BepuThreadingIssueExample.Program.Main(String[] args) in Program.cs:line 59
                */
                Console.WriteLine(ex.ToString());
            }

            Simulation.Statics.Remove(compoundHandle);

            //*******************************
            //*  BIG COMPOUND SWEEP FAILURE *
            //*******************************
            var bigCompoundShape = Simulation.Shapes.Add(new BigCompound(children, Simulation.Shapes, BufferPool));
            compoundHandle = Simulation.Statics.Add(new StaticDescription(RigidPose.Identity, bigCompoundShape));
            try
            {
                var sweeper = new Sphere(5.0f);
                var sweepHandler = new DemoSweepHandler();
                Simulation.Sweep(sweeper, RigidPose.Identity, new BodyVelocity(Vector3.Zero), float.MaxValue, BufferPool, ref sweepHandler); // <Error source
            }
            catch (Exception ex)
            {
                /*
                    System.NotImplementedException: Compounds can never be nested; this should never be called.
                       at BepuPhysics.CollisionDetection.SweepTasks.ConvexHomogeneousCompoundSweepTask`6.PreorderedTypeSweep(Void* shapeDataA, RigidPose& localPoseA, Quaternion orientationA, BodyVelocity& velocityA, Void* shapeDataB, RigidPose& localPoseB, Vector3 offsetB, Quaternion orientationB, BodyVelocity& velocityB, Single maximumT, Single minimumProgression, Single convergenceThreshold, Int32 maximumIterationCount, Single& t0, Single& t1, Vector3& hitLocation, Vector3& hitNormal)
                       at BepuPhysics.CollisionDetection.SweepTask.Sweep(Void* shapeDataA, Int32 shapeTypeA, RigidPose& localPoseA, Quaternion orientationA, BodyVelocity& velocityA, Void* shapeDataB, Int32 shapeTypeB, RigidPose& localPoseB, Vector3 offsetB, Quaternion orientationB, BodyVelocity& velocityB, Single maximumT, Single minimumProgression, Single convergenceThreshold, Int32 maximumIterationCount, Single& t0, Single& t1, Vector3& hitLocation, Vector3& hitNormal)
                       at BepuPhysics.CollisionDetection.SweepTasks.ConvexCompoundSweepTask`4.PreorderedTypeSweep[TSweepFilter](Void* shapeDataA, Quaternion orientationA, BodyVelocity& velocityA, Void* shapeDataB, Vector3 offsetB, Quaternion orientationB, BodyVelocity& velocityB, Single maximumT, Single minimumProgression, Single convergenceThreshold, Int32 maximumIterationCount, Boolean flipRequired, TSweepFilter& filter, Shapes shapes, SweepTaskRegistry sweepTasks, BufferPool pool, Single& t0, Single& t1, Vector3& hitLocation, Vector3& hitNormal)
                       at BepuPhysics.CollisionDetection.SweepTask.Sweep[TSweepFilter](Void* shapeDataA, Int32 shapeTypeA, Quaternion orientationA, BodyVelocity& velocityA, Void* shapeDataB, Int32 shapeTypeB, Vector3 offsetB, Quaternion orientationB, BodyVelocity& velocityB, Single maximumT, Single minimumProgression, Single convergenceThreshold, Int32 maximumIterationCount, TSweepFilter& filter, Shapes shapes, SweepTaskRegistry sweepTasks, BufferPool pool, Single& t0, Single& t1, Vector3& hitLocation, Vector3& hitNormal)
                       at BepuPhysics.Trees.Tree.Sweep[TLeafTester](Vector3 expansion, Vector3 origin, Vector3 direction, TreeRay* treeRay, TLeafTester& sweepTester)
                       at BepuPhysics.CollisionDetection.BroadPhase.Sweep[TSweepTester](Vector3 min, Vector3 max, Vector3 direction, Single maximumT, TSweepTester& sweepTester)
                       at BepuPhysics.Simulation.Sweep[TShape,TSweepHitHandler](TShape shape, RigidPose& pose, BodyVelocity& velocity, Single maximumT, BufferPool pool, TSweepHitHandler& hitHandler, Single minimumProgression, Single convergenceThreshold, Int32 maximumIterationCount)
                       at BepuPhysics.Simulation.Sweep[TShape,TSweepHitHandler](TShape& shape, RigidPose& pose, BodyVelocity& velocity, Single maximumT, BufferPool pool, TSweepHitHandler& hitHandler)
                       at BepuThreadingIssueExample.Program.Main(String[] args) in Program.cs:line 85

                */
                Console.WriteLine(ex.ToString());
            }
        }
    }
}
