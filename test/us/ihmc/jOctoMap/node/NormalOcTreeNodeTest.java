package us.ihmc.jOctoMap.node;

import static org.junit.Assert.*;

import java.util.Random;

import javax.vecmath.Point3d;

import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.junit.Test;

import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.tools.JOctoMapRandomTools;

public class NormalOcTreeNodeTest
{

   @Test
   public void testUpdateHitLocation()
   {
      Random random = new Random(4543L);
      NormalOcTreeNode node = new NormalOcTreeNode();
      Mean xMean = new Mean();
      Mean yMean = new Mean();
      Mean zMean = new Mean();

      Point3d randomPoint = new Point3d();

      for (int i = 0; i < 1000; i++)
      {
         randomPoint.add(JOctoMapRandomTools.generateRandomPoint3d(random, 1.0, 1.0, 1.0));
         node.updateHitLocation(randomPoint, 0);
         xMean.increment(randomPoint.getX());
         yMean.increment(randomPoint.getY());
         zMean.increment(randomPoint.getZ());

         assertEquals(xMean.getResult(), node.getHitLocationX(), 1.0e-3);
         assertEquals(yMean.getResult(), node.getHitLocationY(), 1.0e-3);
         assertEquals(zMean.getResult(), node.getHitLocationZ(), 1.0e-3);
      }
   }

}
