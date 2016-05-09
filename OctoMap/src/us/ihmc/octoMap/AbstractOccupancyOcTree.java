package us.ihmc.octoMap;

import static us.ihmc.octoMap.OctoMapTools.logodds;
import static us.ihmc.octoMap.OctoMapTools.probability;

import javax.vecmath.Point3d;

import us.ihmc.robotics.MathTools;

public abstract class AbstractOccupancyOcTree<NODE extends OcTreeNode> extends OcTreeBaseImpl<Float, NODE>
{
   // occupancy parameters of tree, stored in logodds:
   protected float clamping_thres_min;
   protected float clamping_thres_max;
   protected float prob_hit_log;
   protected float prob_miss_log;
   private float occ_prob_thres_log;

   public AbstractOccupancyOcTree(double resolution)
   {
      super(resolution);
      setDefaultParameters();
   }

   /// Constructor to enable derived classes to change tree constants.
   /// This usually requires a re-implementation of some core tree-traversal functions as well!
   protected AbstractOccupancyOcTree(double resolution, int tree_depth, int tree_max_val)
   {
      super(resolution, tree_depth, tree_max_val);
      setDefaultParameters();
   }

   public AbstractOccupancyOcTree(AbstractOccupancyOcTree<NODE> other)
   {
      super(other);
      clamping_thres_min = other.clamping_thres_min;
      clamping_thres_max = other.clamping_thres_max;
      prob_hit_log = other.prob_hit_log;
      prob_miss_log = other.prob_miss_log;
      occ_prob_thres_log = other.occ_prob_thres_log;
   }

   public void setDefaultParameters()
   {
      // some sane default values:
      setOccupancyThres(0.5);   // = 0.0 in logodds
      setProbHit(0.7);          // = 0.85 in logodds
      setProbMiss(0.4);         // = -0.4 in logodds

      setClampingThresMin(0.1192); // = -2 in log odds
      setClampingThresMax(0.971); // = 3.5 in log odds
   }

   /// queries whether a node is occupied according to the tree's parameter for "occupancy"
   public boolean isNodeOccupied(NODE occupancyNode)
   {
      return (occupancyNode.getLogOdds() >= this.occ_prob_thres_log);
   }

   /// queries whether a node is at the clamping threshold according to the tree's parameter
   public boolean isNodeAtThreshold(NODE occupancyNode)
   {
      return (occupancyNode.getLogOdds() >= this.clamping_thres_max || occupancyNode.getLogOdds() <= this.clamping_thres_min);
   }

   /**
    * Manipulate log_odds value of voxel directly
    *
    * @param key of the NODE that is to be updated
    * @param log_odds_update value to be added (+) to log_odds value of node
    * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   public NODE updateNode(OcTreeKey key, float log_odds_update)
   {
      return updateNode(key, log_odds_update, false);
   }

   public abstract NODE updateNode(OcTreeKey key, float log_odds_update, boolean lazy_eval);

   /**
    * Manipulate log_odds value of voxel directly.
    * Looks up the OcTreeKey corresponding to the coordinate and then calls udpateNode() with it.
    *
    * @param value 3d coordinate of the NODE that is to be updated
    * @param log_odds_update value to be added (+) to log_odds value of node
    * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   public NODE updateNode(Point3d value, float log_odds_update)
   {
      return updateNode(value, log_odds_update, false);
   }

   public abstract NODE updateNode(Point3d value, float log_odds_update, boolean lazy_eval);

   /**
    * Integrate occupancy measurement.
    *
    * @param key of the NODE that is to be updated
    * @param occupied true if the node was measured occupied, else false
    * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   public NODE updateNode(OcTreeKey key, boolean occupied)
   {
      return updateNode(key, occupied, false);
   }

   public abstract NODE updateNode(OcTreeKey key, boolean occupied, boolean lazy_eval);

   /**
    * Integrate occupancy measurement.
    * Looks up the OcTreeKey corresponding to the coordinate and then calls udpateNode() with it.
    *
    * @param value 3d coordinate of the NODE that is to be updated
    * @param occupied true if the node was measured occupied, else false
    * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @return pointer to the updated NODE
    */
   public NODE updateNode(Point3d value, boolean occupied)
   {
      return updateNode(value, occupied, false);
   }

   public abstract NODE updateNode(Point3d value, boolean occupied, boolean lazy_eval);

   public abstract void toMaxLikelihood();

   //-- parameters for occupancy and sensor model:

   /// sets the threshold for occupancy (sensor model)
   public void setOccupancyThres(double prob)
   {
      occ_prob_thres_log = logodds(prob);
   }

   /// sets the probability for a "hit" (will be converted to logodds) - sensor model
   public void setProbHit(double prob)
   {
      prob_hit_log = logodds(prob);
      MathTools.checkIfPositive(prob_hit_log);
   }

   /// sets the probability for a "miss" (will be converted to logodds) - sensor model
   public void setProbMiss(double prob)
   {
      prob_miss_log = logodds(prob);
      MathTools.checkIfNegative(prob_miss_log);
   }

   /// sets the minimum threshold for occupancy clamping (sensor model)
   public void setClampingThresMin(double thresProb)
   {
      clamping_thres_min = logodds(thresProb);
   }

   /// sets the maximum threshold for occupancy clamping (sensor model)
   public void setClampingThresMax(double thresProb)
   {
      clamping_thres_max = logodds(thresProb);
   }

   /// @return threshold (probability) for occupancy - sensor model
   public double getOccupancyThres()
   {
      return probability(occ_prob_thres_log);
   }

   /// @return threshold (logodds) for occupancy - sensor model
   public float getOccupancyThresLog()
   {
      return occ_prob_thres_log;
   }

   /// @return probability for a "hit" in the sensor model (probability)
   public double getProbHit()
   {
      return probability(prob_hit_log);
   }

   /// @return probability for a "hit" in the sensor model (logodds)
   public float getProbHitLog()
   {
      return prob_hit_log;
   }

   /// @return probability for a "miss"  in the sensor model (probability)
   public double getProbMiss()
   {
      return probability(prob_miss_log);
   }

   /// @return probability for a "miss"  in the sensor model (logodds)
   public float getProbMissLog()
   {
      return prob_miss_log;
   }

   /// @return minimum threshold for occupancy clamping in the sensor model (probability)
   public double getClampingThresMin()
   {
      return probability(clamping_thres_min);
   }

   /// @return minimum threshold for occupancy clamping in the sensor model (logodds)
   public float getClampingThresMinLog()
   {
      return clamping_thres_min;
   }

   /// @return maximum threshold for occupancy clamping in the sensor model (probability)
   public double getClampingThresMax()
   {
      return probability(clamping_thres_max);
   }

   /// @return maximum threshold for occupancy clamping in the sensor model (logodds)
   public float getClampingThresMaxLog()
   {
      return clamping_thres_max;
   }

}
