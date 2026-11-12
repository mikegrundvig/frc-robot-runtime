package com.michaelgrundvig.frc.toolconfigs.advantagescope;

/**
 * The two models written for each robot. They are the same robot, with and without its cameras'
 * fields of view: the second is for a copy drawn beside the first (the simulator's truth, as a
 * ghost), where two sets of cones would only crowd the view.
 */
enum SchematicModel {
  /** The robot with its cameras' fields of view, and its cameras as views to look through. */
  WITH_CONES,
  /** The robot alone. */
  WITHOUT_CONES;

  /** The folder AdvantageScope reads this model of a robot from. */
  String folder(String robot) {
    return switch (this) {
      case WITH_CONES -> AdvantageScopeAssets.folder(robot);
      case WITHOUT_CONES -> AdvantageScopeAssets.withoutConesFolder(robot);
    };
  }

  /** This model's name, as AdvantageScope lists it to choose for a robot. */
  String modelName(String robot) {
    return switch (this) {
      case WITH_CONES -> AdvantageScopeAssets.modelName(robot);
      case WITHOUT_CONES -> AdvantageScopeAssets.withoutConesModelName(robot);
    };
  }
}
