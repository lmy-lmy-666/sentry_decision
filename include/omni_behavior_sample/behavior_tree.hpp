// Copyright 2026 Boombroke
//
// A tiny, dependency-free behaviour-tree kernel.
// ================================================
// Just enough of the classic BT vocabulary to express the sentry decision:
//   - NodeStatus            SUCCESS / FAILURE / RUNNING
//   - Sequence              run children L→R, stop at first non-SUCCESS
//   - Fallback (Selector)   run children L→R, stop at first non-FAILURE
//   - ReactiveSequence      like Sequence but re-tick from the FIRST child
//                           every tick (so an earlier guard can abort a
//                           RUNNING later child)
//   - ReactiveFallback      like Fallback but re-tick from the FIRST child
//                           every tick (priority arbitration: a higher-priority
//                           branch can preempt a RUNNING lower-priority one)
//   - Inverter              flip SUCCESS<->FAILURE, pass RUNNING through
//   - Condition             leaf wrapping a bool() functor
//   - Action                leaf wrapping a NodeStatus() functor
//
// Design notes
// ------------
// * Header-only, no ROS, no BehaviorTree.CPP — keeps the port buildable on any
//   ROS distro (tested target: Jazzy) with zero extra apt packages, and lets
//   the whole decision be unit-tested exactly like the FSM sample.
// * The tree is ticked at a fixed rate (10 Hz) by the node. Long-running
//   behaviours (drive to a waypoint, dash across a bump) return RUNNING and
//   keep their own progress state in the blackboard between ticks.
// * "Reactive" composites are what give the tree the same every-tick
//   re-evaluation the FSM's priority chain had: the highest-priority runnable
//   branch always wins, even mid-action.
//
#ifndef OMNI_BEHAVIOR_SAMPLE__BEHAVIOR_TREE_HPP_
#define OMNI_BEHAVIOR_SAMPLE__BEHAVIOR_TREE_HPP_

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace omni_behavior_sample
{
namespace bt
{

enum class NodeStatus { SUCCESS, FAILURE, RUNNING };

/// Base class for every node. `name_` is purely for logging / tree dumps.
class TreeNode
{
public:
  explicit TreeNode(std::string name) : name_(std::move(name)) {}
  virtual ~TreeNode() = default;

  virtual NodeStatus tick() = 0;

  /// Reset any latched/running progress (called when a parent abandons us).
  /// Default: no-op; composites override to propagate to children.
  virtual void halt() {}

  const std::string & name() const { return name_; }

protected:
  std::string name_;
};

using TreeNodePtr = std::unique_ptr<TreeNode>;

// ===========================================================================
//  leaves
// ===========================================================================

/// A condition leaf: evaluate a predicate → SUCCESS (true) / FAILURE (false).
/// Conditions are pure and instantaneous — they never return RUNNING.
class ConditionNode : public TreeNode
{
public:
  ConditionNode(std::string name, std::function<bool()> pred)
  : TreeNode(std::move(name)), pred_(std::move(pred)) {}

  NodeStatus tick() override
  {
    return pred_() ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

private:
  std::function<bool()> pred_;
};

/// An action leaf: run a functor that returns a NodeStatus. Actions may return
/// RUNNING across ticks; they own their progress state externally (blackboard).
class ActionNode : public TreeNode
{
public:
  ActionNode(std::string name, std::function<NodeStatus()> fn)
  : TreeNode(std::move(name)), fn_(std::move(fn)) {}

  NodeStatus tick() override { return fn_(); }

private:
  std::function<NodeStatus()> fn_;
};

// ===========================================================================
//  composites
// ===========================================================================

class ControlNode : public TreeNode
{
public:
  explicit ControlNode(std::string name) : TreeNode(std::move(name)) {}

  ControlNode * add(TreeNodePtr child)
  {
    children_.push_back(std::move(child));
    return this;
  }

  void halt() override
  {
    for (auto & c : children_) c->halt();
  }

protected:
  std::vector<TreeNodePtr> children_;
};

/// Fallback (a.k.a. Selector), REACTIVE: every tick re-evaluates children from
/// the first one. Returns SUCCESS/RUNNING at the first child that yields it;
/// only if every child FAILs does the fallback FAIL.
///
/// This is the workhorse for priority arbitration: list branches high→low
/// priority; the first one that is applicable (returns SUCCESS/RUNNING) wins,
/// and a higher-priority branch becoming applicable preempts a lower one that
/// was RUNNING — exactly the FSM priority-chain semantics.
class ReactiveFallback : public ControlNode
{
public:
  explicit ReactiveFallback(std::string name) : ControlNode(std::move(name)) {}

  NodeStatus tick() override
  {
    for (std::size_t i = 0; i < children_.size(); ++i) {
      const NodeStatus s = children_[i]->tick();
      if (s != NodeStatus::FAILURE) {
        // A child took the tick; halt every LOWER-priority child so any
        // progress they had latched is cleared (they were preempted).
        for (std::size_t j = i + 1; j < children_.size(); ++j) children_[j]->halt();
        return s;
      }
    }
    return NodeStatus::FAILURE;
  }
};

/// Sequence, REACTIVE: every tick re-ticks from the first child. Returns
/// FAILURE/RUNNING at the first child that yields it; SUCCESS only if all
/// children SUCCEED. Used for "guard(s) then body": if an earlier guard flips
/// to FAILURE it aborts a later child that was RUNNING.
class ReactiveSequence : public ControlNode
{
public:
  explicit ReactiveSequence(std::string name) : ControlNode(std::move(name)) {}

  NodeStatus tick() override
  {
    for (std::size_t i = 0; i < children_.size(); ++i) {
      const NodeStatus s = children_[i]->tick();
      if (s != NodeStatus::SUCCESS) {
        for (std::size_t j = i + 1; j < children_.size(); ++j) children_[j]->halt();
        return s;
      }
    }
    return NodeStatus::SUCCESS;
  }
};

/// Plain (non-reactive) Sequence: advances an internal cursor, remembering
/// which child is RUNNING so it does not re-tick earlier children. Resets on
/// SUCCESS/FAILURE or halt. Provided for completeness; the decision tree uses
/// the reactive variants for its every-tick re-evaluation.
class SequenceNode : public ControlNode
{
public:
  explicit SequenceNode(std::string name) : ControlNode(std::move(name)) {}

  NodeStatus tick() override
  {
    while (cursor_ < children_.size()) {
      const NodeStatus s = children_[cursor_]->tick();
      if (s == NodeStatus::RUNNING) return NodeStatus::RUNNING;
      if (s == NodeStatus::FAILURE) { reset(); return NodeStatus::FAILURE; }
      ++cursor_;  // SUCCESS → next child
    }
    reset();
    return NodeStatus::SUCCESS;
  }

  void halt() override { reset(); ControlNode::halt(); }

private:
  void reset() { cursor_ = 0; }
  std::size_t cursor_{0};
};

// ===========================================================================
//  decorators
// ===========================================================================

class DecoratorNode : public TreeNode
{
public:
  DecoratorNode(std::string name, TreeNodePtr child)
  : TreeNode(std::move(name)), child_(std::move(child)) {}

  void halt() override { if (child_) child_->halt(); }

protected:
  TreeNodePtr child_;
};

/// Inverter: SUCCESS<->FAILURE, RUNNING passes through.
class InverterNode : public DecoratorNode
{
public:
  InverterNode(std::string name, TreeNodePtr child)
  : DecoratorNode(std::move(name), std::move(child)) {}

  NodeStatus tick() override
  {
    switch (child_->tick()) {
      case NodeStatus::SUCCESS: return NodeStatus::FAILURE;
      case NodeStatus::FAILURE: return NodeStatus::SUCCESS;
      default:                  return NodeStatus::RUNNING;
    }
  }
};

/// ForceSuccess: map FAILURE→SUCCESS (RUNNING passes through). Handy to wrap a
/// branch whose "I did something" outcome should not fail its parent.
class ForceSuccessNode : public DecoratorNode
{
public:
  ForceSuccessNode(std::string name, TreeNodePtr child)
  : DecoratorNode(std::move(name), std::move(child)) {}

  NodeStatus tick() override
  {
    const NodeStatus s = child_->tick();
    return (s == NodeStatus::RUNNING) ? NodeStatus::RUNNING : NodeStatus::SUCCESS;
  }
};

// ===========================================================================
//  builders — small helpers so tree construction reads top-down
// ===========================================================================

inline TreeNodePtr Condition(std::string name, std::function<bool()> pred)
{
  return std::make_unique<ConditionNode>(std::move(name), std::move(pred));
}

inline TreeNodePtr Action(std::string name, std::function<NodeStatus()> fn)
{
  return std::make_unique<ActionNode>(std::move(name), std::move(fn));
}

inline TreeNodePtr Inverter(std::string name, TreeNodePtr child)
{
  return std::make_unique<InverterNode>(std::move(name), std::move(child));
}

inline TreeNodePtr ForceSuccess(std::string name, TreeNodePtr child)
{
  return std::make_unique<ForceSuccessNode>(std::move(name), std::move(child));
}

/// Variadic composite builders: `MakeReactiveFallback("root", a, b, c)`.
template <typename NodeT, typename... Children>
inline std::unique_ptr<NodeT> MakeComposite(std::string name, Children &&... kids)
{
  auto node = std::make_unique<NodeT>(std::move(name));
  (node->add(std::forward<Children>(kids)), ...);
  return node;
}

template <typename... Children>
inline TreeNodePtr MakeReactiveFallback(std::string name, Children &&... kids)
{
  return MakeComposite<ReactiveFallback>(std::move(name), std::forward<Children>(kids)...);
}

template <typename... Children>
inline TreeNodePtr MakeReactiveSequence(std::string name, Children &&... kids)
{
  return MakeComposite<ReactiveSequence>(std::move(name), std::forward<Children>(kids)...);
}

template <typename... Children>
inline TreeNodePtr MakeSequence(std::string name, Children &&... kids)
{
  return MakeComposite<SequenceNode>(std::move(name), std::forward<Children>(kids)...);
}

}  // namespace bt
}  // namespace omni_behavior_sample

#endif  // OMNI_BEHAVIOR_SAMPLE__BEHAVIOR_TREE_HPP_
