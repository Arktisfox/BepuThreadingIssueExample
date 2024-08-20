using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities.Memory;
using System.Numerics;

namespace BepuThreadingIssueExample
{
    internal class Program
    {
        static BufferPool? BufferPool;
        static Simulation? Simulation;

        static void Main(string[] args)
        {
            BufferPool = new BufferPool();
            Simulation = Simulation.Create(BufferPool, new NoCollisionCallbacks(), new EmptyPoseIntegratorCallbacks(), new SolveDescription(8, 1));

            
            BufferPool.Take<Triangle>(2, out var triBuffer);

            ref var triA = ref triBuffer[0];
            triA.A = new Vector3(-1, 0, -1);
            triA.B = new Vector3(1, 0, -1);
            triA.C = new Vector3(1, 0, 1);

            ref var triB = ref triBuffer[0];
            triA.A = new Vector3(-1, 0, -1);
            triA.B = new Vector3(1, 0, 11);
            triA.C = new Vector3(-1, 0, 1);

            var groundplane = new Mesh(triBuffer, Vector3.One, BufferPool);
            Simulation.Statics.Add(new StaticDescription(RigidPose.Identity, Simulation.Shapes.Add(groundplane)));

            Parallel.For(0, 10000, (i) =>
            {
                try
                {
                    var sweeper = new Sphere(5.0f);
                    var sweepHandler = new DemoSweepHandler();
                    var rayHandler = new DemoRayHandler();

                    Simulation.RayCast(Vector3.UnitY, -Vector3.UnitY, float.MaxValue, ref rayHandler);
                    Simulation.Sweep(sweeper, RigidPose.Identity, new BodyVelocity(Vector3.Zero), float.MaxValue, BufferPool, ref sweepHandler);
                }
                catch(Exception ex)
                {
                    Console.WriteLine(ex.ToString());
                }
            });
        }
    }
}
