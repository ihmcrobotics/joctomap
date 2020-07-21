package us.ihmc.jOctoMap.tools;

import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.apache.commons.math3.stat.descriptive.moment.SecondMoment;

/**
 * Computes the variance of the available values.  By default, the unbiased
 * "sample variance" definitional formula is used:
 * <p>
 * variance = sum((x_i - mean)^2) / (n - 1) </p>
 * <p>
 * where mean is the {@link Mean} and <code>n</code> is the number
 * of sample observations.</p>
 * <p>
 * This uses the Welford algorithm for updating the second moment incrementally, and it uses the
 * Chan algorithm for updating it as a block. These can be found:
 * * <a href="https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance"> Algorithms for calculating variance </a>.
 */
public class IncrementalVariance
{
   /**
    * Whether or not bias correction is applied when computing the
    * value of the statistic. True means that bias is corrected.
    */
   private final boolean isBiasCorrected;

   /**
    * second moment of values that have been added
    */
   private double m2;

   /**
    * Count of values that have been added
    */
   private long n;

   /**
    * First moment of values that have been added
    */
   private double mean;

   public IncrementalVariance()
   {
      this(true);
   }

   public IncrementalVariance(boolean isBiasCorrected)
   {
      this.isBiasCorrected = isBiasCorrected;
   }

   public void set(double mean, double m2, long size)
   {
      this.mean = mean;
      this.m2 = m2;
      this.n = size;
   }

   public void increment(double d)
   {
      if (n < 1)
      {
         mean = m2 = 0.0;
      }

      if (n == 0)
      {
         mean = 0.0;
      }
      n++;

      double dev = d - mean;
      double nDev = dev / n;
      mean += nDev;

      m2 += (d - mean) * dev;
   }

   public void increment(double d, long numberOfSamples)
   {
      if (n < 1)
      {
         mean = m2 = 0.0;
      }

      if (n == 0)
      {
         mean = 0.0;
      }
      long nPrev = n;
      n += numberOfSamples;

      double dev = d - mean;
      double nDev = numberOfSamples * dev / n;
      mean += nDev;

      m2 += dev * dev * numberOfSamples * nPrev / n;
   }

   public double getMean()
   {
       return mean;
   }

   public double getVariance()
   {
      if (n == 0)
      {
         return Double.NaN;
      }
      else if (n == 1)
      {
         return 0d;
      }
      else
      {
         if (isBiasCorrected)
         {
            return m2 / (n - 1d);
         }
         else
         {
            return m2 / (n);
         }
      }
   }

   public long getN()
   {
      return n;
   }

   public void clear()
   {
      m2 = Double.NaN;
      mean = Double.NaN;
      n = 0;
   }

   public boolean isBiasCorrected()
   {
      return isBiasCorrected;
   }
}
