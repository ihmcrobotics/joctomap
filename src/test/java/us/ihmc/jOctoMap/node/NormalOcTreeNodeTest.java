package us.ihmc.jOctoMap.node;

import static us.ihmc.robotics.Assert.assertEquals;

import java.util.Random;

import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Point3D;
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

      Point3D randomPoint = new Point3D();

      for (int i = 0; i < 1000; i++)
      {
         randomPoint.add(JOctoMapRandomTools.generateRandomPoint3D(random, 1.0, 1.0, 1.0));
         node.updateHitLocation(randomPoint);
         xMean.increment(randomPoint.getX());
         yMean.increment(randomPoint.getY());
         zMean.increment(randomPoint.getZ());

         assertEquals(xMean.getResult(), node.getHitLocationX(), 1.0e-3);
         assertEquals(yMean.getResult(), node.getHitLocationY(), 1.0e-3);
         assertEquals(zMean.getResult(), node.getHitLocationZ(), 1.0e-3);
      }
   }

}
