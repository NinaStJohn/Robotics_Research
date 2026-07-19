#include "sensing.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

#include "debug_log.hpp"

std::vector<Pos> sense(const SensorConfig& cfg, Pos center, const GridWorld& world)
{
    std::vector<Pos> result;
    int r = static_cast<int>(std::ceil(cfg.radius));

    for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
            Pos p{center.x + dx, center.y + dy};
            if (!world.in_bounds(p)) continue;

            double dist;
            switch (cfg.metric) {
                case SensorMetric::Chebyshev:
                    dist = static_cast<double>(std::max(std::abs(dx), std::abs(dy)));
                    break;
                case SensorMetric::Manhattan:
                    dist = static_cast<double>(std::abs(dx) + std::abs(dy));
                    break;
                case SensorMetric::Euclidean:
                default:
                    dist = std::sqrt(static_cast<double>(dx * dx + dy * dy));
                    break;
            }
            if (dist <= cfg.radius) result.push_back(p);
        }
    }

    dbg("sensing.log") << "[sense] center=(" << center.x << "," << center.y << ")"
                        << " radius=" << cfg.radius
                        << " visible=" << result.size() << "\n" << std::flush;

    return result;
}

std::vector<Pos> reveal(GridWorld& robot_map, const GridWorld& world,
                         const std::vector<Pos>& visible)
{
    std::vector<Pos> diff;

    for (const Pos& p : visible) {
        bool was_discovered = robot_map.is_discovered(p);
        robot_map.mark_discovered(p);

        bool truth  = world.is_dynamic(p);
        bool belief = robot_map.is_dynamic(p);

        if (truth != belief) {
            robot_map.set_dynamic(p, truth);
            diff.push_back(p);

            dbg("sensing.log") << "[reveal] cell=(" << p.x << "," << p.y << ")"
                                << " first_seen=" << (!was_discovered)
                                << " belief " << belief << " -> " << truth << "\n";
        }
    }

    if (!diff.empty())
        dbg("sensing.log") << "[reveal] " << diff.size() << " cell(s) changed belief\n" << std::flush;

    return diff;
}

GridWorld seed_from_static(const GridWorld& world)
{
    GridWorld robot_map(world.width(), world.height());

    for (const std::string& name : world.label_names()) {
        for (int y = 0; y < world.height(); ++y) {
            for (int x = 0; x < world.width(); ++x) {
                Pos p{x, y};
                if (world.has_label(p, name))
                    robot_map.set_label(p, name, true);
            }
        }
    }

    for (int y = 0; y < world.height(); ++y) {
        for (int x = 0; x < world.width(); ++x) {
            Pos p{x, y};
            if (world.is_static(p)) {
                robot_map.set_static(p, true);
                robot_map.mark_discovered(p);
            }
        }
    }

    dbg("sensing.log") << "[seed_from_static] robot_map seeded "
                        << world.width() << "x" << world.height() << "\n" << std::flush;

    return robot_map;
}

std::vector<StateChange> build_state_changes(
    const WPA& wpa,
    const GridWorld& belief,
    const std::vector<unsigned>& changed_states
){
    std::vector<StateChange> changes;
    changes.reserve(changed_states.size());

    for (unsigned u : changed_states) {
        double cold        = wpa.state_exit_weight(u);
        bool   now_blocked = belief.is_blocked(wpa.pos_of(u));
        double new_cost    = now_blocked ? std::numeric_limits<double>::infinity() : 1.0;

        if (cold == new_cost) continue;   // no actual change for this state

        changes.push_back(StateChange{u, cold, new_cost});

        dbg("sensing.log") << "[state_change] state=" << u
                            << " pos=(" << wpa.pos_of(u).x << "," << wpa.pos_of(u).y << ")"
                            << " cold=" << cold << " new_cost=" << new_cost << "\n";
    }

    return changes;
}
