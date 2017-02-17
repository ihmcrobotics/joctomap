package us.ihmc.jOctoMap.tools;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

/**
 * This class provides a storeless computation for a 3D covariance matrix.
 * Implementation from the algorithm described on Wikipedia:
 * <a href="https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance"> Algorithms for calculating variance </a>.
 * <p>
 * It seems that there is a debate on whether this should be named the covariance or variance matrix, see
 * <a href="https://en.wikipedia.org/wiki/Covariance_matrix"> Covariance matrix </a>
 * <p>
 * This class is a tool to compute the two following versions of the covariance matrix:
 * <ul>
 *   <li> Standard Covariance matrix: CoVar(X) = Y<sup>T</sup> * Y / n
 *   <li> Covariance matrix with <a href="https://en.wikipedia.org/wiki/Bessel%27s_correction"> Bessel's Correction </a>: CoVar(X) = Y<sup>T</sup> * Y / (n-1)
 *   <li>Where:
 *   <ul>
 *     <li> Y = X - <b>1</b> mean(X)
 *     <li> n is the size of the dataset
 *     <li> X represents the n-by-3 dataset
 *     <li> <b>1</b> is a n-by-1 vector filled with 1.
 *     <li> mean(X) is the 1-by-3 average vector of the dataset X.
 *   </ul>
 * </ul>
 * @author Sylvain
 *
 */
public class IncrementalCovariance3D
{
   private int sampleSize = 0;
   private final Point3D mean = new Point3D();
   private final DenseMatrix64F secondMoment = new DenseMatrix64F(3, 3);

   public IncrementalCovariance3D()
   {
   }

   /**
    * Clear the current data.
    * If the mean of the next dataset is somewhat known, it is preferable to use {@link #clearAndSetPredictedMean(Tuple3d)}.
    */
   public void clear()
   {
      sampleSize = 0;
      mean.set(0.0, 0., 0.0);
      secondMoment.zero();
   }

   /**
    * Inserts a new data point (x, y, z) and updates the covariance matrix.
    * @param x the x-coordinate of the new data point.
    * @param y the y-coordinate of the new data point.
    * @param z the z-coordinate of the new data point.
    */
   public void addDataPoint(double x, double y, double z)
   {
      sampleSize++;
      double devX = x - mean.getX();
      double devY = y - mean.getY();
      double devZ = z - mean.getZ();
      double nInv = 1.0 / sampleSize;

      mean.setX(mean.getX() + devX * nInv);
      mean.setY(mean.getY() + devY * nInv);
      mean.setZ(mean.getZ() + devZ * nInv);

      // Using the known symmetricity of the covariance matrix.
      double m00 = devX * (x - mean.getX());
      double m11 = devY * (y - mean.getY());
      double m22 = devZ * (z - mean.getZ());
      double m01 = devX * (y - mean.getY());
      double m02 = devX * (z - mean.getZ());
      double m12 = devY * (z - mean.getZ());

      secondMoment.add(0, 0, m00);
      secondMoment.add(0, 1, m01);
      secondMoment.add(0, 2, m02);
      secondMoment.add(1, 0, m01);
      secondMoment.add(1, 1, m11);
      secondMoment.add(1, 2, m12);
      secondMoment.add(2, 0, m02);
      secondMoment.add(2, 1, m12);
      secondMoment.add(2, 2, m22);
   }

   /**
    * Get the the average of the current dataset.
    * @param meanToPack
    */
   public void getMean(Tuple3DBasics meanToPack)
   {
      meanToPack.set(mean);
   }

   /**
    * Get the covariance matrix corresponding to the dataset added beforehand.
    * @return the 3-by-3 covariance matrix.
    */
   public DenseMatrix64F getCovariance()
   {
      DenseMatrix64F covariance = new DenseMatrix64F(3, 3);
      double div = 1.0 / (double) (sampleSize);
      covariance.set(secondMoment);
      CommonOps.scale(div, covariance);
      return covariance;
   }

   /**
    * Get the covariance matrix corresponding to the dataset added beforehand using <a href="https://en.wikipedia.org/wiki/Bessel%27s_correction"> Bessel's Correction </a>.
    * @return the 3-by-3 covariance matrix.
    */
   public DenseMatrix64F getCovarianceCorrected()
   {
      DenseMatrix64F covariance = new DenseMatrix64F(3, 3);
      double div = 1.0 / (double) (sampleSize - 1.0);
      covariance.set(secondMoment);
      CommonOps.scale(div, covariance);
      return covariance;
   }

   /**
    * @return the current number of data points.
    */
   public int getSampleSize()
   {
      return sampleSize;
   }
}
