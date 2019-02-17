using NUnit.Framework;
using OMPLUnity;

namespace Tests
{
    [TestFixture]
    public class Tests
    {
        [TestCase(-1.0, 1.0)]
        [TestCase(-99.0, -50.0)]
        [TestCase(500.0, 599.0)]
        public void TestAddDimension(double min, double max)
        {
            OMPL.AddDimension(min, max);
            Assert.AreEqual(OMPL.DimensionCount, 1);
        }

        [Test]
        public void TestReset()
        {
            OMPL.AddDimension(-1.0, 1.0);
            Assert.AreEqual(1, OMPL.DimensionCount);
            OMPL.Reset();
            Assert.AreEqual(0, OMPL.DimensionCount);
        }

        [Test]
        public void TestAddDimensions()
        {
            OMPL.AddDimension(-1.0, 1.0);
            OMPL.AddDimension(-99.0, -50.0);
            OMPL.AddDimension(500.0, 599.0);
            Assert.AreEqual(3, OMPL.DimensionCount);  
        }

        [Test]
        public void TestSolve()
        {
            OMPL.AddDimension(-1.0, 1.0);
            OMPL.AddDimension(-59.0, -33.0);
            OMPL.AddDimension(0.0, 500.0);
            double[] initial = new double[] { -.5, -34.0, 0.0 };
            double[] goal = new double[] { .9, -49.0, 500.0 };

            OMPL.SetValidityChecker((double[] state, int length) =>
            {
                return true;
            });

            double[,] solution = OMPL.Solve(initial, goal, 5.0);

            for(int i=0; i<solution.GetLength(0); i++)
            {
                for(int j=0; j<solution.GetLength(1); j++)
                {
                    TestContext.Out.Write(solution[i, j] + ", ");
                }
                TestContext.Out.WriteLine();
            }
            
        }

        [Test]
        public void TestSetValidityChecker()
        {
            Assert.IsFalse(OMPL.HasSetValidityChecker());

            OMPL.SetValidityChecker((double[] state, int length) =>
            {
                return true;
            });

            Assert.IsTrue(OMPL.HasSetValidityChecker());
        }


        [TearDown]
        public void TearDown()
        {
            OMPL.Reset();
        }
    }

}