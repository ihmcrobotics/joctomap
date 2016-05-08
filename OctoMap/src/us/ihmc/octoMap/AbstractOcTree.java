package us.ihmc.octoMap;

public abstract class AbstractOcTree
{

   public AbstractOcTree()
   {
   }

   /// virtual constructor: creates a new object of same type
   public abstract AbstractOcTree create();

   /// returns actual class name as string for identification
   public abstract String getTreeType();

   public abstract double getResolution();

   public abstract void setResolution(double res);

   public abstract int size();

   public abstract int memoryUsage();

   public abstract int memoryUsageNode();

   public abstract void getMetricMin(double x, double y, double z);

   public abstract void getMetricMax(double x, double y, double z);

   public abstract void getMetricSize(double x, double y, double z);

   public abstract void prune();

   public abstract void expand();

   public abstract void clear();
}
