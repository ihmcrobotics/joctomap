package us.ihmc.octoMap.key;

import java.util.HashMap;

/**
 * Data structrure to efficiently track changed nodes as a combination of
 * OcTreeKeys and a bool flag (to denote newly created nodes)
 *
 */
public class KeyBoolMap extends HashMap<OcTreeKey, Boolean>
{
   private static final long serialVersionUID = 2656234567169415329L;
}