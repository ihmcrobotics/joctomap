package us.ihmc.jOctoMap.tools;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import us.ihmc.robotics.Assert;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;


public class IncrementalCovariance3DTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testEasyCase()
   {
      Random random = new Random(51651L);
      IncrementalCovariance3D incrementalCovariance3D = new IncrementalCovariance3D();

      Point3D average = new Point3D();
      int length = 100;
      Vector3D maxAmplitude = new Vector3D(1.0, 1.0, 1.0);

      for (int i = 0; i < 100; i++)
      {
         List<Point3D> dataset = createRandomDataset(random, average, length, maxAmplitude);

         incrementalCovariance3D.clear();
         dataset.forEach(point -> incrementalCovariance3D.addDataPoint(point.getX(), point.getY(), point.getZ()));
         assertCovarianceIsCorrect(incrementalCovariance3D, dataset);
      }
   }

   @Test
   public void testNonZeroMean()
   {
      Random random = new Random(51651L);
      IncrementalCovariance3D incrementalCovariance3D = new IncrementalCovariance3D();

      int length = 100;
      Vector3D maxAmplitude = new Vector3D(1.0, 1.0, 1.0);

      for (int i = 0; i < 100; i++)
      {
         Point3D average = JOctoMapRandomTools.generateRandomPoint3D(random, 10.0, 10.0, 10.0);
         List<Point3D> dataset = createRandomDataset(random, average, length, maxAmplitude);

         incrementalCovariance3D.clear();
         dataset.forEach(point -> incrementalCovariance3D.addDataPoint(point.getX(), point.getY(), point.getZ()));
         assertCovarianceIsCorrect(incrementalCovariance3D, dataset);
      }
   }

   private void assertCovarianceIsCorrect(IncrementalCovariance3D incrementalCovariance3D, List<Point3D> dataset)
   {
      Assert.assertEquals(dataset.size(), incrementalCovariance3D.getSampleSize());

      Point3D expectedMean = new Point3D();
      dataset.forEach(point -> expectedMean.scaleAdd(1.0 / dataset.size(), point, expectedMean));
      Point3D actualMean = new Point3D();
      incrementalCovariance3D.getMean(actualMean);
      assertTrue(expectedMean.epsilonEquals(actualMean, EPSILON));

      DenseMatrix64F expectedCovariance;
      DenseMatrix64F actualCovariance = incrementalCovariance3D.getCovariance();
      expectedCovariance = computeCovarianceMatrix(dataset, false);
      assertEquals(expectedCovariance, actualCovariance, EPSILON);

      actualCovariance = incrementalCovariance3D.getCovarianceCorrected();
      expectedCovariance = computeCovarianceMatrix(dataset, true);
      assertEquals(expectedCovariance, actualCovariance, EPSILON);
   }

   private void assertEquals(DenseMatrix64F expectedCovariance, DenseMatrix64F actualCovariance, double epsilon)
   {
      assertTrue(assertErrorMessage(expectedCovariance, actualCovariance), MatrixFeatures.isEquals(expectedCovariance, actualCovariance, epsilon));
   }

   private static String assertErrorMessage(DenseMatrix64F expectedCovariance, DenseMatrix64F actualCovariance)
   {
      return "Expected:\n" + expectedCovariance + "\nActual:\n" + actualCovariance;
   }

   private static List<Point3D> createRandomDataset(Random random, Point3D average, int length, Vector3D maxAmplitude)
   {
      List<Point3D> dataset = new ArrayList<>(length);
      Point3D min = new Point3D();
      Point3D max = new Point3D();

      min.sub(average, maxAmplitude);
      max.add(average, maxAmplitude);

      for (int i = 0; i < length; i++)
         dataset.add(JOctoMapRandomTools.generateRandomPoint3D(random, min, max));

      return dataset;
   }

   /**
    * Using the actual formula of the covariance matrix, <a href="https://en.wikipedia.org/wiki/Principal_component_analysis"> here</a>.
    */
   private static DenseMatrix64F computeCovarianceMatrix(List<Point3D> dataset, boolean corrected)
   {
      DenseMatrix64F covariance = new DenseMatrix64F(3, 3);
      int n = dataset.size();
      DenseMatrix64F datasetMatrix = new DenseMatrix64F(n, 3);

      Point3D average = new Point3D();
      dataset.forEach(point -> average.scaleAdd(1.0 / dataset.size(), point, average));

      for (int i = 0; i < n; i++)
      {
         Point3D dataPoint = dataset.get(i);
         datasetMatrix.set(i, 0, dataPoint.getX() - average.getX());
         datasetMatrix.set(i, 1, dataPoint.getY() - average.getY());
         datasetMatrix.set(i, 2, dataPoint.getZ() - average.getZ());
      }

      CommonOps.multInner(datasetMatrix, covariance);

      if (corrected)
      {
         CommonOps.scale(1.0 / (double) (n - 1.0), covariance);
      }
      else
      {
         CommonOps.scale(1.0 / (double) n, covariance);
      }

      return covariance;
   }
}
