/*
    Replanning test suite (headless, no raylib).

    Two layers of checking run after every (re)plan:

      TIER 1  validate_lasso()  — structural invariants that must always hold:
              * path[0] is the robot's current cell
              * no prefix/cycle cell is blocked
              * consecutive cells (incl. the cycle wrap) are 4-adjacent
              * the cycle anchor is an accepting state that owns a cycle

      TIER 2  cross-check        — incremental result must match a from-scratch
              rebuild: g(current) from the incremental planner == g(current)
              from a fresh make_planner() on the same (mutated) WPA. This is the
              strong optimality check; it catches stale g-fields / stale cycles.

    Plus a seeded random fuzz that hammers both layers.

    Build:  make tests      Run:  ./replan_tests.out [fuzz_seed] [fuzz_steps]
    Exit code is the number of failing checks (0 = all good).

    All [DBG] chatter from the planner is redirected to a sink; test output
    (TEST/PASS/FAIL/SUMMARY) goes to std::cerr so it stays readable.
*/

#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "robot.hpp"
#include "grid_world.hpp"
#include "ts.hpp"
#include "dstar.hpp"
#include "wpa.hpp"

static const std::string LTL = "G(a -> Fb) & G(b -> Fa)";

// ------------------------------------------------------------------
// Helpers
// ------------------------------------------------------------------

// Standard 6x6 test world: a@(0,0), b@(5,5), no interior obstacles
// (scenarios add their own, so each starts from a known clean state).
static GridWorld make_world() {
    GridWorld world(6, 6);
    world.set_label({0, 0}, "a", true);
    world.set_label({5, 5}, "b", true);
    return world;
}

static int manhattan(Pos a, Pos b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

struct Flat {
    std::vector<Pos>      pos;
    std::vector<unsigned> ids;
    int                   cycle_start;   // index the loop restarts at (the anchor)
};

static void flat_append(
    std::vector<Pos>& pos, std::vector<unsigned>& ids,
    const std::vector<Pos>& seg, const std::vector<unsigned>& seg_ids
){
    for (size_t i = 0; i < seg.size(); ++i) {
        if (!pos.empty() && pos.back().x == seg[i].x && pos.back().y == seg[i].y)
            continue;
        pos.push_back(seg[i]);
        ids.push_back(seg_ids[i]);
    }
}

// Same flattening + cycle-restart logic as grid_vis.cpp: prefix ends at the
// anchor and the cycle begins at that same anchor, so the loop restart index
// is (deduped prefix length - 1).
static Flat flatten(const LassoResult& l) {
    Flat f;
    flat_append(f.pos, f.ids, l.prefix, l.prefix_ids);
    int pref_len = (int)f.pos.size();
    flat_append(f.pos, f.ids, l.cycle, l.cycle_ids);
    f.cycle_start = pref_len > 0 ? pref_len - 1 : 0;
    return f;
}

static std::string cell(Pos p) {
    std::ostringstream os;
    os << "(" << p.x << "," << p.y << ")";
    return os.str();
}

// ------------------------------------------------------------------
// TIER 1 — structural invariants
// ------------------------------------------------------------------
static bool validate_lasso(
    const GridWorld& world,
    const WPA& wpa,
    const DStarPlanner& planner,
    const LassoResult& lasso,
    Pos robot_cell,
    std::string& err
){
    Flat f = flatten(lasso);

    if (f.pos.empty()) { err = "empty lasso"; return false; }

    // path starts at the robot
    if (f.pos[0].x != robot_cell.x || f.pos[0].y != robot_cell.y) {
        err = "path[0]=" + cell(f.pos[0]) + " != robot " + cell(robot_cell);
        return false;
    }

    // no cell is blocked
    for (Pos p : f.pos) {
        if (world.is_blocked(p)) { err = "path crosses blocked " + cell(p); return false; }
    }

    // consecutive cells are grid-adjacent
    for (size_t i = 1; i < f.pos.size(); ++i) {
        if (manhattan(f.pos[i - 1], f.pos[i]) != 1) {
            err = "non-adjacent step " + cell(f.pos[i - 1]) + "->" + cell(f.pos[i]);
            return false;
        }
    }

    // the cycle wraps adjacently back to the anchor
    if (f.cycle_start < (int)f.pos.size()) {
        Pos last = f.pos.back();
        Pos anchor = f.pos[f.cycle_start];
        if (manhattan(last, anchor) > 1) {
            err = "cycle wrap " + cell(last) + "->" + cell(anchor) + " not adjacent";
            return false;
        }
    }

    // the anchor is an accepting state that actually owns a cycle
    unsigned anchor_id = f.ids[f.cycle_start];
    if (!wpa.is_accepting(anchor_id)) {
        err = "anchor state " + std::to_string(anchor_id) + " not accepting";
        return false;
    }
    if (planner.cycle_path.count(anchor_id) == 0) {
        err = "anchor state " + std::to_string(anchor_id) + " has no cycle";
        return false;
    }

    return true;
}

// ------------------------------------------------------------------
// TIER 2 — incremental vs from-scratch optimality
//
// g(current) is the total cost-to-go (descend to best anchor + close its
// cycle). A correct incremental update must match a full rebuild on the same
// edge weights. inf == inf is a valid (consistent infeasible) agreement.
// ------------------------------------------------------------------
static bool cross_check(WPA& wpa, const DStarPlanner& inc, unsigned current,
                        double& g_inc, double& g_full, DStarPlanner& fresh_out) {
    g_inc = get_g(inc.prefix.g, current);
    fresh_out = make_planner(wpa, ReplanMode::DSTAR_INCREMENTAL);
    g_full = get_g(fresh_out.prefix.g, current);
    return g_inc == g_full;   // exact: values are integer-valued or both inf
}

// On a g mismatch, decide whether the discrepancy lives in the suffix (a stale
// cycle_cost = the Option-1 gap) or the prefix g-field (a real incremental bug).
static void diagnose(const WPA& wpa, const DStarPlanner& inc,
                     const DStarPlanner& fresh) {
    std::set<unsigned> anchors;
    for (std::unordered_map<unsigned, double>::const_iterator it = inc.cycle_cost.begin();
         it != inc.cycle_cost.end(); ++it) anchors.insert(it->first);
    for (std::unordered_map<unsigned, double>::const_iterator it = fresh.cycle_cost.begin();
         it != fresh.cycle_cost.end(); ++it) anchors.insert(it->first);

    bool cycle_mismatch = false;
    for (unsigned acc : anchors) {
        double inf = std::numeric_limits<double>::infinity();
        double ci = inc.cycle_cost.count(acc)   ? inc.cycle_cost.at(acc)   : inf;
        double cf = fresh.cycle_cost.count(acc)  ? fresh.cycle_cost.at(acc) : inf;
        if (ci != cf) {
            cycle_mismatch = true;
            std::cerr << "    cycle_cost[anchor " << acc << " " << cell(wpa.pos_of(acc))
                      << "]  inc=" << ci << "  fresh=" << cf << "  MISMATCH\n";
        }
    }
    if (cycle_mismatch)
        std::cerr << "    => SUFFIX stale: Option-1 only re-searches loops a changed cell"
                     " sits on, so an unblock that opens a cheaper loop is missed."
                     " Fixed by Option-2 SUFFIXREPLAN.\n";
    else
        std::cerr << "    => cycle_cost all match; discrepancy is in the PREFIX g-field"
                     " (real incremental bug to fix).\n";
}

// ------------------------------------------------------------------
// Scenario runner
// ------------------------------------------------------------------
struct Toggle { int x, y; };

// Returns the number of failed checks in this scenario.
static int run_scenario(const std::string& name,
                        const std::vector<Toggle>& toggles,
                        int robot_steps) {
    int fails = 0;
    std::cerr << "TEST [" << name << "]\n";

    GridWorld world = make_world();
    Turtlebot bot({0, 0});
    ProductBundle bundle = build_product_from_world_robot_ltl(world, bot, LTL);
    WPA wpa(std::move(bundle));
    DStarPlanner planner = make_planner(wpa, ReplanMode::DSTAR_INCREMENTAL);

    unsigned start = wpa.state_of(bot.position(), wpa.nba_init_state());
    LassoResult lasso = dstar_plan(wpa, planner, start);

    // place the robot some steps along the initial path, then replan from there
    Flat f0 = flatten(lasso);
    int step = robot_steps;
    if (step < 0) step = 0;
    if (step > (int)f0.pos.size() - 1) step = (int)f0.pos.size() - 1;
    unsigned current = f0.ids[step];
    Pos robot_cell = wpa.pos_of(current);

    lasso = dstar_plan(wpa, planner, current);   // baseline lasso from robot
    std::string err;
    if (!validate_lasso(world, wpa, planner, lasso, robot_cell, err)) {
        std::cerr << "  FAIL baseline: " << err << "\n";
        ++fails;
    }

    for (const Toggle& t : toggles) {
        Pos c{t.x, t.y};
        world.set_blocked(c, !world.is_blocked(c));
        bool now_blocked = world.is_blocked(c);
        std::vector<unsigned> changed = detect_changed_states(wpa, {c});

        lasso = dstar_replan(wpa, planner, current, changed, now_blocked, planner.mode);

        // Tier 2 first: does incremental agree with a full rebuild?
        double g_inc = 0, g_full = 0;
        DStarPlanner fresh;
        bool agree = cross_check(wpa, planner, current, g_inc, g_full, fresh);
        std::string tag = std::string("toggle ") + cell(c) +
                          (now_blocked ? " BLOCK" : " FREE");
        if (!agree) {
            std::cerr << "  FAIL " << tag << ": g_inc=" << g_inc
                      << " != g_full=" << g_full << " (suboptimal/stale)\n";
            diagnose(wpa, planner, fresh);
            ++fails;
        }

        // Tier 1: structure — only meaningful when a path exists
        bool feasible = g_full != std::numeric_limits<double>::infinity();
        if (feasible) {
            if (!validate_lasso(world, wpa, planner, lasso, robot_cell, err)) {
                std::cerr << "  FAIL " << tag << ": " << err << "\n";
                ++fails;
            }
        } else {
            // infeasible: incremental must also report no path
            if (!lasso.prefix.empty() || !lasso.cycle.empty()) {
                std::cerr << "  FAIL " << tag
                          << ": task infeasible but lasso non-empty\n";
                ++fails;
            }
        }
    }

    if (fails == 0) std::cerr << "  PASS\n";
    return fails;
}

// ------------------------------------------------------------------
// Random fuzz: random toggles, both tiers checked each step.
// Skips the two label cells so we mostly exercise navigation reroutes.
// ------------------------------------------------------------------
static int run_fuzz(unsigned seed, int steps) {
    int fails = 0;
    std::cerr << "TEST [fuzz seed=" << seed << " steps=" << steps << "]\n";

    GridWorld world = make_world();
    Turtlebot bot({0, 0});
    ProductBundle bundle = build_product_from_world_robot_ltl(world, bot, LTL);
    WPA wpa(std::move(bundle));
    DStarPlanner planner = make_planner(wpa, ReplanMode::DSTAR_INCREMENTAL);
    unsigned current = wpa.state_of(bot.position(), wpa.nba_init_state());
    Pos robot_cell = wpa.pos_of(current);
    dstar_plan(wpa, planner, current);

    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> coord(0, 5);

    for (int i = 0; i < steps; ++i) {
        Pos c{coord(rng), coord(rng)};
        if ((c.x == 0 && c.y == 0) || (c.x == 5 && c.y == 5)) continue; // skip labels
        world.set_blocked(c, !world.is_blocked(c));
        bool now_blocked = world.is_blocked(c);
        std::vector<unsigned> changed = detect_changed_states(wpa, {c});

        LassoResult lasso =
            dstar_replan(wpa, planner, current, changed, now_blocked, planner.mode);

        double g_inc = 0, g_full = 0;
        DStarPlanner fresh;
        bool agree = cross_check(wpa, planner, current, g_inc, g_full, fresh);
        if (!agree) {
            std::cerr << "  FAIL step " << i << " toggle " << cell(c)
                      << (now_blocked ? " BLOCK" : " FREE")
                      << ": g_inc=" << g_inc << " != g_full=" << g_full << "\n";
            diagnose(wpa, planner, fresh);
            ++fails;
            break;   // stop at first divergence; rerun with this seed to debug
        }

        bool feasible = g_full != std::numeric_limits<double>::infinity();
        std::string err;
        if (feasible) {
            if (!validate_lasso(world, wpa, planner, lasso, robot_cell, err)) {
                std::cerr << "  FAIL step " << i << " toggle " << cell(c)
                          << ": " << err << "\n";
                ++fails;
                break;
            }
        } else if (!lasso.prefix.empty() || !lasso.cycle.empty()) {
            std::cerr << "  FAIL step " << i << " toggle " << cell(c)
                      << ": infeasible but lasso non-empty\n";
            ++fails;
            break;
        }
    }

    if (fails == 0) std::cerr << "  PASS\n";
    return fails;
}

// ------------------------------------------------------------------
int main(int argc, char** argv) {
    // silence planner [DBG] output (cout); test output goes to cerr
    std::ostringstream sink;
    std::streambuf* old_cout = std::cout.rdbuf(sink.rdbuf());

    int fails = 0;

    // Tier 4 — pinned regression scenarios (robot at path start unless noted)
    fails += run_scenario("block on-path (5,2)",        {{5, 2}},          0);
    fails += run_scenario("block cycle-only (3,1)",     {{3, 1}},          0);
    fails += run_scenario("block off-path (2,3)",       {{2, 3}},          0);
    fails += run_scenario("block then unblock (5,2)",   {{5, 2}, {5, 2}},  0);
    fails += run_scenario("multi-block right column",   {{5, 2}, {5, 1}, {5, 3}}, 0);
    fails += run_scenario("mid-path replan @step5",     {{5, 2}},          5);

    // Tier 3 — fuzz (seed + steps overridable from argv)
    unsigned seed  = (argc > 1) ? (unsigned)std::stoul(argv[1]) : 1u;
    int      steps = (argc > 2) ? std::stoi(argv[2]) : 60;
    fails += run_fuzz(seed, steps);

    std::cout.rdbuf(old_cout);
    std::cerr << "\nSUMMARY: " << (fails == 0 ? "ALL PASS" : "FAILURES")
              << " (" << fails << " failing check(s))\n";
    return fails;
}
