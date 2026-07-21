// Copyright 2026 Boombroke
//
// Unit tests for the dependency-free behaviour-tree kernel (behavior_tree.hpp).
// Verifies the composite/decorator semantics the decision tree relies on:
//   ReactiveFallback priority preemption, ReactiveSequence guard-abort,
//   Inverter, and leaf conditions/actions.
//
#include "omni_behavior_sample/behavior_tree.hpp"

#include <gtest/gtest.h>

using namespace omni_behavior_sample::bt;

// -----------------------------------------------------------------------------
//  Leaves
// -----------------------------------------------------------------------------

TEST(BtKernel, ConditionTrueIsSuccess)
{
  ConditionNode c{"c", [] { return true; }};
  EXPECT_EQ(c.tick(), NodeStatus::SUCCESS);
}

TEST(BtKernel, ConditionFalseIsFailure)
{
  ConditionNode c{"c", [] { return false; }};
  EXPECT_EQ(c.tick(), NodeStatus::FAILURE);
}

TEST(BtKernel, ActionReturnsItsStatus)
{
  ActionNode a{"a", [] { return NodeStatus::RUNNING; }};
  EXPECT_EQ(a.tick(), NodeStatus::RUNNING);
}

// -----------------------------------------------------------------------------
//  ReactiveFallback — priority arbitration
// -----------------------------------------------------------------------------

TEST(BtKernel, FallbackReturnsFirstNonFailure)
{
  int ran = 0;
  auto fb = MakeReactiveFallback(
    "fb",
    Condition("c1", [] { return false; }),                 // FAILURE → skip
    Action("a2", [&ran] { ++ran; return NodeStatus::SUCCESS; }),
    Action("a3", [] { return NodeStatus::SUCCESS; }));
  EXPECT_EQ(fb->tick(), NodeStatus::SUCCESS);
  EXPECT_EQ(ran, 1);   // stopped at the first non-FAILURE child
}

TEST(BtKernel, FallbackAllFailIsFailure)
{
  auto fb = MakeReactiveFallback(
    "fb",
    Condition("c1", [] { return false; }),
    Condition("c2", [] { return false; }));
  EXPECT_EQ(fb->tick(), NodeStatus::FAILURE);
}

TEST(BtKernel, FallbackReactivePreemptsLowerPriorityRunning)
{
  // A high-priority branch that starts FAILing (so a lower RUNNING branch wins),
  // then becomes applicable → must preempt the lower branch, whose halt() fires.
  bool high_ready = false;
  bool low_halted = false;

  struct LowNode : TreeNode
  {
    bool * halted;
    explicit LowNode(bool * h) : TreeNode("low"), halted(h) {}
    NodeStatus tick() override { return NodeStatus::RUNNING; }
    void halt() override { *halted = true; }
  };

  auto fb = std::make_unique<ReactiveFallback>("fb");
  fb->add(Condition("high", [&high_ready] { return high_ready; }));
  fb->add(std::make_unique<LowNode>(&low_halted));

  EXPECT_EQ(fb->tick(), NodeStatus::RUNNING);   // high fails → low runs
  EXPECT_FALSE(low_halted);

  high_ready = true;
  EXPECT_EQ(fb->tick(), NodeStatus::SUCCESS);   // high preempts
  EXPECT_TRUE(low_halted) << "lower-priority RUNNING branch must be halted on preemption";
}

// -----------------------------------------------------------------------------
//  ReactiveSequence — guard then body
// -----------------------------------------------------------------------------

TEST(BtKernel, SequenceRunsAllOnSuccess)
{
  int count = 0;
  auto seq = MakeReactiveSequence(
    "seq",
    Action("a1", [&count] { ++count; return NodeStatus::SUCCESS; }),
    Action("a2", [&count] { ++count; return NodeStatus::SUCCESS; }));
  EXPECT_EQ(seq->tick(), NodeStatus::SUCCESS);
  EXPECT_EQ(count, 2);
}

TEST(BtKernel, SequenceStopsAtFirstNonSuccess)
{
  int reached = 0;
  auto seq = MakeReactiveSequence(
    "seq",
    Condition("guard", [] { return false; }),   // FAILURE aborts
    Action("body", [&reached] { ++reached; return NodeStatus::SUCCESS; }));
  EXPECT_EQ(seq->tick(), NodeStatus::FAILURE);
  EXPECT_EQ(reached, 0) << "body must not run when the guard fails";
}

TEST(BtKernel, SequenceGuardAbortsRunningBody)
{
  bool guard_ok = true;
  auto seq = MakeReactiveSequence(
    "seq",
    Condition("guard", [&guard_ok] { return guard_ok; }),
    Action("body", [] { return NodeStatus::RUNNING; }));
  EXPECT_EQ(seq->tick(), NodeStatus::RUNNING);
  guard_ok = false;
  EXPECT_EQ(seq->tick(), NodeStatus::FAILURE)
      << "reactive sequence re-checks the guard and aborts a running body";
}

// -----------------------------------------------------------------------------
//  Inverter
// -----------------------------------------------------------------------------

TEST(BtKernel, InverterFlipsSuccessAndFailure)
{
  auto inv_s = Inverter("i", Condition("c", [] { return true; }));
  EXPECT_EQ(inv_s->tick(), NodeStatus::FAILURE);
  auto inv_f = Inverter("i", Condition("c", [] { return false; }));
  EXPECT_EQ(inv_f->tick(), NodeStatus::SUCCESS);
}

TEST(BtKernel, InverterPassesRunningThrough)
{
  auto inv = Inverter("i", Action("a", [] { return NodeStatus::RUNNING; }));
  EXPECT_EQ(inv->tick(), NodeStatus::RUNNING);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
