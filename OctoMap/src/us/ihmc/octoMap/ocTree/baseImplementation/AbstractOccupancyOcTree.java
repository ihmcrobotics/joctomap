package us.ihmc.octoMap.ocTree.baseImplementation;

import static us.ihmc.octoMap.tools.OctoMapTools.logodds;
import static us.ihmc.octoMap.tools.OctoMapTools.probability;

import javax.vecmath.Point3d;

import us.ihmc.octoMap.key.OcTreeKey;
import us.ihmc.octoMap.node.AbstractOccupancyOcTreeNode;
import us.ihmc.robotics.MathTools;

public abstract class AbstractOccupancyOcTree<NODE extends AbstractOccupancyOcTreeNode<NODE>> extends AbstractOcTreeBase<NODE>
{
   // occupancy parameters of tree, stored in logodds:
   protected float minOccupancyLogOdds;
   protected float maxOccupancyLogOdds;
   protected float hitUpdateLogOdds;
   protected float missUpdateLogOdds;
   private float occupancyThresholdLogOdds;

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
      minOccupancyLogOdds = other.minOccupancyLogOdds;
      maxOccupancyLogOdds = other.maxOccupancyLogOdds;
      hitUpdateLogOdds = other.hitUpdateLogOdds;
      missUpdateLogOdds = other.missUpdateLogOdds;
      occupancyThresholdLogOdds = other.occupancyThresholdLogOdds;
   }

   public void setDefaultParameters()
   {
      // some sane default values:
      setOccupancyThreshold(0.5);            // = 0.0 in logodds
      setHitProbabilityUpdate(0.7);          // = 0.85 in logodds
      setMissProbabilityUpdate(0.4);         // = -0.4 in logodds

      setMinProbability(0.1192);             // = -2 in log odds
      setMaxProbability(0.971);              // = 3.5 in log odds
   }

   /**
    * Queries whether a node is occupied according to the tree's parameter for "occupancyThreshold"
    * @param occupancyNode
    * @return
    */
   public boolean isNodeOccupied(NODE occupancyNode)
   {
      return occupancyNode.getLogOdds() >= this.occupancyThresholdLogOdds;
   }

   /// queries whether a node is at the clamping threshold according to the tree's parameter
   public boolean isNodeAtThreshold(NODE occupancyNode)
   {
      return occupancyNode.getLogOdds() >= this.maxOccupancyLogOdds || occupancyNode.getLogOdds() <= this.minOccupancyLogOdds;
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

   /**
    * Sets the threshold for occupancy (sensor model)
    * @param probability
    */
   public void setOccupancyThreshold(double probability)
   {
      occupancyThresholdLogOdds = logodds(probability);
   }

   /**
    * Sets the probability for a "hit" (will be converted to logodds) - sensor model
    * @param probability
    */
   public void setHitProbabilityUpdate(double probability)
   {
      hitUpdateLogOdds = logodds(probability);
      MathTools.checkIfPositive(hitUpdateLogOdds);
   }

   /**
    * Sets the probability for a "miss" (will be converted to logodds) - sensor model
    * @param probabillity
    */
   public void setMissProbabilityUpdate(double probabillity)
   {
      missUpdateLogOdds = logodds(probabillity);
      MathTools.checkIfNegative(missUpdateLogOdds);
   }

   /**
    * Sets the minimum probability for occupancy clamping (sensor model)
    * @param minimumProbability
    */
   public void setMinProbability(double minimumProbability)
   {
      minOccupancyLogOdds = logodds(minimumProbability);
   }

   /**
    * Sets the maximum probability for occupancy clamping (sensor model)
    * @param maximumProbability
    */
   public void setMaxProbability(double maximumProbability)
   {
      maxOccupancyLogOdds = logodds(maximumProbability);
   }

   /**
    * @return threshold (probability) for occupancy - sensor model
    */
   public double getOccupancyThreshold()
   {
      return probability(occupancyThresholdLogOdds);
   }

   /**
    * @return threshold (logodds) for occupancy - sensor model
    */
   public float getOccupancyThresholdLogOdds()
   {
      return occupancyThresholdLogOdds;
   }

   /**
    * @return probability for a "hit" in the sensor model (probability)
    */
   public double getHitProbability()
   {
      return probability(hitUpdateLogOdds);
   }

   /**
    * @return probability for a "hit" in the sensor model (logodds)
    */
   public float getHitProbabilityLogOdds()
   {
      return hitUpdateLogOdds;
   }

   /**
    * @return probability for a "miss"  in the sensor model (probability)
    */
   public double getMissProbability()
   {
      return probability(missUpdateLogOdds);
   }

   /**
    * @return probability for a "miss"  in the sensor model (logodds)
    */
   public float getMissProbabilityLogOdds()
   {
      return missUpdateLogOdds;
   }

   /**
    * @return minimum probability for occupancy clamping in the sensor model
    */
   public double getMinProbability()
   {
      return probability(minOccupancyLogOdds);
   }

   /**
    * @return minimum logodds for occupancy clamping in the sensor model
    */
   public float getMinLogOdds()
   {
      return minOccupancyLogOdds;
   }

   /**
    * @return maximum probability for occupancy clamping in the sensor model
    */
   public double getMaxProbability()
   {
      return probability(maxOccupancyLogOdds);
   }

   /**
    * @return maximum logodds for occupancy clamping in the sensor model
    */
   public float getMaxLogOdds()
   {
      return maxOccupancyLogOdds;
   }

}
