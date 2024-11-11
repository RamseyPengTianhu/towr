#include <towr/initialization/quadruped_to_biped_gait_generator.h>
#include <cassert>
#include <iostream>

namespace towr
{

  QuadrupedToBipedGaitGenerator::QuadrupedToBipedGaitGenerator()
  {
    Q_ = {true, true, true, true};     // quadrupedal stance (all legs in contact)
    T_ = {false, false, true, true};   // transition stance (only back legs in contact)
    B_ = {false, false, true, true};   // bipedal stance (back legs in contact)
    I_ = {false, false, false, false}; // flight (no contact)

    SetGaits({Stand, Transition, BipedStand});
  }

  void QuadrupedToBipedGaitGenerator::SetCombo(Combos combo)
  {
    switch (combo)
    {
    case C0:
      SetGaits({Stand, Transition, BipedStand});
    case C1:
      SetGaits({Stand, Transition, BipedStand});
      break;
    case C2:
      SetGaits({Stand, Stand, Stand});
      break;
    case C3:
      SetGaits({Transition, Transition, Transition});
      break;
    case C4:
      SetGaits({BipedStand, BipedStand, BipedStand});
    case C5:
      SetGaits({Stand, BipedStand, BipedStand});

      break;
    default:
      assert(false);
      std::cout << "Gait not defined\n";
      break;
    }
  }

  GaitGenerator::GaitInfo QuadrupedToBipedGaitGenerator::GetGait(Gaits gait) const
  {
    switch (gait)
    {
    case Stand:
      return GetQuadrupedStand();
    case Transition:
      return GetTransitionToBiped();
    case BipedStand:
      return GetBipedStand();
    default:
      assert(false); // gait not implemented
    }
  }

  GaitGenerator::GaitInfo QuadrupedToBipedGaitGenerator::GetQuadrupedStand() const
  {
    auto times = {0.3, 0.3};
    auto contacts = {Q_, T_};
    return std::make_pair(times, contacts);
  }

  GaitGenerator::GaitInfo QuadrupedToBipedGaitGenerator::GetTransitionToBiped() const
  {
    auto times = {0.3, 0.3};
    auto contacts = {Q_, T_};
    return std::make_pair(times, contacts);
  }

  GaitGenerator::GaitInfo QuadrupedToBipedGaitGenerator::GetBipedStand() const
  {
    auto times = {0.3};
    auto contacts = {B_};
    return std::make_pair(times, contacts);
  }

} // namespace towr