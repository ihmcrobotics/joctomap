package us.ihmc.jOctoMap.tools;

import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class IncrementalVarianceTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testEasyCase()
   {
      Random random = new Random(1738L);
      IncrementalVariance incrementalVariance = new IncrementalVariance();

      double average = 0.0;
      int length = 100;
      double maxAmplitude = 1.0;;

      for (int i = 0; i < 100; i++)
      {
         double[] dataset = createRandomDataset(random, average, length, maxAmplitude);

         incrementalVariance.clear();
         for (double data : dataset)
            incrementalVariance.increment(data);
         assertCovarianceIsCorrect(incrementalVariance, dataset);
      }
   }

   @Test
   public void testEasyMultiplePoints()
   {
      IncrementalVariance incrementalVariance = new IncrementalVariance();

      double point1 = 1.0;
      double point2 = 2.0;
      double point3 = 6.0;
      double point4 = 7.0;

      double[] dataset = new double[11];
      dataset[0] = point1;
      dataset[1] = point1;
      dataset[2] = point1;
      dataset[3] = point2;
      dataset[4] = point2;
      dataset[5] = point3;
      dataset[6] = point3;
      dataset[7] = point3;
      dataset[8] = point3;
      dataset[9] = point4;
      dataset[10] = point4;


      incrementalVariance.clear();
      incrementalVariance.increment(point1, 3);
      incrementalVariance.increment(point2, 2);
      incrementalVariance.increment(point3, 4);
      incrementalVariance.increment(point4, 2);
      assertCovarianceIsCorrect(incrementalVariance, dataset);

   }

   @Test
   public void testNonZeroMean()
   {
      Random random = new Random(1738L);
      IncrementalVariance incrementalVariance = new IncrementalVariance();

      int length = 100;
      double maxAmplitude = 1.0;

      for (int i = 0; i < 100; i++)
      {
         double average = -10.0 + random.nextDouble() * (20.0);
         double[] dataset = createRandomDataset(random, average, length, maxAmplitude);

         incrementalVariance.clear();
         Arrays.stream(dataset).forEach(incrementalVariance::increment);
         assertCovarianceIsCorrect(incrementalVariance, dataset);
      }
   }

   private void assertCovarianceIsCorrect(IncrementalVariance incrementalCovariance, double[] dataset)
   {
      double expectedMean = Arrays.stream(dataset).average().getAsDouble();
      assertEquals(expectedMean, incrementalCovariance.getMean(), EPSILON);
      assertEquals(computeVariance(dataset, incrementalCovariance.isBiasCorrected()), incrementalCovariance.getVariance(), EPSILON);
   }


   private static double[] createRandomDataset(Random random, double average, int length, double maxAmplitude)
   {
      double[] dataset = new double[length];
      double min = average - maxAmplitude;
      double max = average + maxAmplitude;

      for (int i = 0; i < length; i++)
         dataset[i] = min + random.nextDouble() * (max - min);

      return dataset;
   }

   /**
    * Using the actual formula of the covariance matrix, <a href="https://en.wikipedia.org/wiki/Principal_component_analysis"> here</a>.
    */
   private static double computeVariance(double[] dataset, boolean corrected)
   {
      double mean = Arrays.stream(dataset).average().getAsDouble();
      double m2 = 0.0;
      for (double value : dataset)
         m2 += (value - mean) * (value - mean);
      int n = dataset.length;

      double variance;
      if (corrected)
      {
         variance = m2 / (n - 1);
      }
      else
      {
         variance = m2 / n;
      }

      return variance;
   }
}
