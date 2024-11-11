#ifndef TOWR_MODELS_QUADRUPED_TO_BIPED_GAIT_GENERATOR_H_
#define TOWR_MODELS_QUADRUPED_TO_BIPED_GAIT_GENERATOR_H_

#include "gait_generator.h"

namespace towr
{

  /**
   * @brief Produces the contact sequence for transitioning from a quadrupedal to a bipedal gait.
   *
   * This class defines the sequence of phases for the transition from standing on four legs to standing on two back legs.
   *
   * @sa GaitGenerator for more documentation
   */
  class QuadrupedToBipedGaitGenerator : public GaitGenerator
  {
  public:
    QuadrupedToBipedGaitGenerator();
    virtual ~QuadrupedToBipedGaitGenerator() = default;

  private:
    GaitInfo GetGait(Gaits gait) const override;

    GaitInfo GetQuadrupedStand() const;
    GaitInfo GetTransitionToBiped() const;
    GaitInfo GetBipedStand() const;

    void SetCombo(Combos combo) override;

    // Naming convention: Contact states for the transition
    ContactState I_; // flight (no contact)
    ContactState Q_; // quadrupedal stance (all legs in contact)
    ContactState T_; // transition stance (only back legs in contact)
    ContactState B_; // bipedal stance (back legs in contact)
  };

} /* namespace towr */

#endif /* TOWR_MODELS_QUADRUPED_TO_BIPED_GAIT_GENERATOR_H_ */