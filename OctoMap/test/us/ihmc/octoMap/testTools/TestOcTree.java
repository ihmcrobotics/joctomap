package us.ihmc.octoMap.testTools;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.key.OcTreeKeyReadOnly;
import us.ihmc.octoMap.ocTree.baseImplementation.AbstractOcTreeBase;
import us.ihmc.octoMap.ocTree.rules.interfaces.UpdateRule;

public class TestOcTree extends AbstractOcTreeBase<TestOcTreeNode>
{
   private final UpdateRule<TestOcTreeNode> doNothingRule = new UpdateRule<TestOcTreeNode>()
   {
      @Override
      public void updateLeaf(TestOcTreeNode leafToUpdate, OcTreeKeyReadOnly leafKey, boolean nodeJustCreated)
      {
      }

      @Override
      public void updateInnerNode(TestOcTreeNode innerNodeToUpdate)
      {
      }
   };

   public TestOcTree(double resolution, int treeDepth)
   {
      super(resolution, treeDepth);
   }

   public TestOcTree(TestOcTree other)
   {
      super(other);
   }

   public void insertNode(OcTreeKeyReadOnly key)
   {
      updateNodeInternal(key, doNothingRule, null);
   }

   public void insertNode(Point3d coordinate)
   {
      updateNodeInternal(coordinate, doNothingRule, null);
   }

   public void fillRandomly(Random random, int numberOfLeavesToCreate)
   {
      while (getNumLeafNodes() < numberOfLeavesToCreate)
      {
         OcTreeKey randomKey = new OcTreeKey(random, treeDepth, treeDepth);
         insertNode(randomKey);
      }
   }

   public void fillRandomlyWithinSphere(Random random, int numberOfLeavesToCreate, Point3d sphereCenter, double sphereRadius)
   {
      while (getNumLeafNodes() < numberOfLeavesToCreate)
      {
         Vector3d randomTranslation = new Vector3d(random.nextDouble() - 0.5, random.nextDouble() - 0.5, random.nextDouble() - 0.5);
         randomTranslation.scale(sphereRadius * random.nextDouble() / randomTranslation.length());
         Point3d randomCoordinateInSphere = new Point3d();
         randomCoordinateInSphere.add(sphereCenter, randomTranslation);
         insertNode(randomCoordinateInSphere);
      }
   }

   @Override
   protected Class<TestOcTreeNode> getNodeClass()
   {
      return TestOcTreeNode.class;
   }
}
