package us.ihmc.octoMap.boundingBox;

import static org.junit.Assert.*;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.tools.OcTreeKeyConversionTools;

public class OcTreeBoundingBoxWithCenterAndYawTest 
{
	double resolution = 0.025;
	int treeDepth = 16;
	double yaw = Math.PI/2;
	Point3d offset = new Point3d(-150.0, 150.0, 0.0);
	
	Point3d minCoordinate = new Point3d(0.0, 0.0, 0.0);
	Point3d maxCoordinate = new Point3d(100.0, 200.0, 100.0);
	OcTreeKey minKey = new OcTreeKey(32768,32768,32768);
	OcTreeKey maxKey = new OcTreeKey(36768, 40768, 36768);
	
//	Point3d pointA = new Point3d(-150.0, 200.0, 50.0);
//	OcTreeKey keyB = OcTreeKeyConversionTools.coordinateToKey(pointA, resolution, treeDepth);
//	System.out.println(keyB);
	
	@Test
	public void isPointInRotatedBoundingBoxTest()
	{
		Point3d pointA = new Point3d(-200.0, 200.0, 50.0);
		OcTreeBoundingBoxWithCenterAndYaw boundingBox = new OcTreeBoundingBoxWithCenterAndYaw(minCoordinate, maxCoordinate, resolution, treeDepth);
		boundingBox.setYaw(yaw);
		boundingBox.setOffsetCoordinate(offset);
		
		assertTrue(boundingBox.isInBoundingBox(pointA));
	}
	
	@Test 
	public void isKeyInSimpleBoundingBoxTest()
	{
		OcTreeKey keyA = new OcTreeKey(34000, 39000, 34000);
		OcTreeSimpleBoundingBox boundingBox = new OcTreeSimpleBoundingBox(minKey, maxKey);
		boundingBox.update(resolution, treeDepth);
		
		assertTrue(boundingBox.isInBoundingBox(keyA));
	}
	
	@Test 
	public void isKeyInRotatedBoundingBoxTest()
	{
		//OcTreeKey key = new OcTreeKey(29940, 35596, 34768); //45 degrees yaw 
		OcTreeKey key = new OcTreeKey(29304, 34768, 34768); //60 degrees yaw 
		OcTreeSimpleBoundingBox simpleBoundingBox = new OcTreeSimpleBoundingBox(minKey, maxKey);
		simpleBoundingBox.update(resolution, treeDepth);
		
		OcTreeBoundingBoxWithCenterAndYaw boundingBox = new OcTreeBoundingBoxWithCenterAndYaw(simpleBoundingBox, resolution, treeDepth);
		boundingBox.setYaw(Math.PI/3);

		assertTrue(boundingBox.trialIsInBoundingBox(key));
	}
	
	@Test
	public void isKeyInRotatedAndOffsetBoundingBoxTest()
	{
		OcTreeKey key = new OcTreeKey(23940, 41596, 34768); //45 degrees yaw and offset (-150.0, 150.0, 0.0)
		OcTreeSimpleBoundingBox simpleBoundingBox = new OcTreeSimpleBoundingBox(minKey, maxKey);
		simpleBoundingBox.update(resolution, treeDepth);
		
		OcTreeBoundingBoxWithCenterAndYaw boundingBox = new OcTreeBoundingBoxWithCenterAndYaw(simpleBoundingBox, resolution, treeDepth);
		boundingBox.setYaw(Math.PI/4);
		boundingBox.setOffsetCoordinate(offset);

		assertTrue(boundingBox.trialIsInBoundingBox(key));
	}
	

	@Test
	public void isKeyInBoundingBoxVaryingTreeDepthTest()
	{
		OcTreeKey key = new OcTreeKey(23940, 41596, 34768); //45 degrees yaw and offset (-150.0, 150.0, 0.0)
		OcTreeSimpleBoundingBox simpleBoundingBox = new OcTreeSimpleBoundingBox(minKey, maxKey);
		simpleBoundingBox.update(resolution, treeDepth);
		
		OcTreeBoundingBoxWithCenterAndYaw boundingBox = new OcTreeBoundingBoxWithCenterAndYaw(simpleBoundingBox, resolution, treeDepth);
		boundingBox.setYaw(Math.PI/4);
		boundingBox.setOffsetCoordinate(offset);
		
		Point3d pointA = new Point3d(1, 0, 0);
		OcTreeKey keyB = OcTreeKeyConversionTools.coordinateToKey(pointA, resolution, 10);
		System.out.println(keyB);
//		OcTreeKey keyB = new OcTreeKey(16384, 16384, 16384);
//		Point3d coord = OcTreeKeyConversionTools.keyToCoordinate(keyB, resolution, 15);
//		System.out.println(coord);

		assertTrue(boundingBox.trialIsInBoundingBox(key));
	}
}
