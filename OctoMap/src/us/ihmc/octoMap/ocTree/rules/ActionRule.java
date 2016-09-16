package us.ihmc.octoMap.ocTree.rules;

import us.ihmc.octoMap.key.OcTreeKeyReadOnly;

public interface ActionRule
{
   public void doAction(OcTreeKeyReadOnly key);
}
