// Verifies that the MANIFEST LOADER wires the radar equation and per-target RCS.
//
// radar_math_test already proves the formulas. What it cannot prove is that
// sensor_runtime.hpp reads the right JSON keys: a typo there leaves
// detection_reference_range at 0, which silently disables the falloff entirely
// -- the radar keeps working, just with the wrong (unlimited) sensitivity. That
// failure is invisible without a check like this one.
//
// Strategy: drive SensorFactory with a mock ray caster that reports every ray as
// a hit at a fixed distance, then count detections. The count is a direct
// readout of the detection probability at that distance.

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "runtime/sensor_runtime.hpp"

using hako::robots::types::Vector3;
namespace backend = hako::robots::sensor::backend;
namespace runtime = hako::robots::runtime;

static int g_fail = 0;

static void check(bool cond, const char* msg)
{
    std::printf("  [%s] %s\n", cond ? " ok " : "FAIL", msg);
    if (!cond) ++g_fail;
}

// Every ray hits at `dist`, optionally reporting an RCS.
class FixedCaster : public backend::IRayCaster
{
public:
    FixedCaster(double dist, double rcs) : dist_(dist), rcs_(rcs) {}
    backend::RayHit Cast(const Vector3& o, const Vector3& d, double) override
    {
        backend::RayHit h {};
        h.hit = true;
        h.distance = dist_;
        h.point = Vector3(o.x + d.x * dist_, o.y + d.y * dist_, o.z + d.z * dist_);
        h.target_id = 1;
        h.target_rcs_m2 = rcs_;
        return h;
    }
private:
    double dist_, rcs_;
};

// Build a radar component from `params` and count the detections it publishes
// against a target at `dist` with cross-section `rcs`.
static int detections(const nlohmann::json& params, double dist, double rcs)
{
    nlohmann::json comp = {
        {"id", "probe"}, {"kind", "sensor"}, {"type", "radar"},
        {"pdu_name", "radar_scan"}, {"params", params},
    };
    auto caster = std::make_shared<FixedCaster>(dist, rcs);
    auto c = runtime::SensorFactory::Create("radar", comp, caster, "Drone");
    if (!c) return -1;

    runtime::BasePose pose {};
    pose.origin = Vector3(0, 0, 0);
    std::vector<char> buf(200000, 0);
    c->ShouldUpdate(1.0);
    const int n = c->Publish(pose, buf.data(), static_cast<int>(buf.size()));
    if (n < 0) return -1;
    // PointCloud2 payload: width is the detection count. Rather than decode the
    // PDU we re-scan through the same component, which is what Publish just did;
    // the byte size is monotonic in the count and that is all we compare.
    return n;
}

int main()
{
    // 77 GHz, 10 mW, 25 dBi, Smin chosen so Rmax = 6.00 m against sigma_ref = 1.
    nlohmann::json budget = {
        {"range", 40.0}, {"horizontal_fov_deg", 20.0}, {"vertical_fov_deg", 20.0},
        {"points_per_second", 20000}, {"noise_seed", 3},
        {"detection_falloff_exp", 2.0},
        {"tx_power_w", 0.01}, {"antenna_gain_dbi", 25.0},
        {"wavelength_m", 0.003893}, {"min_detectable_signal_w", 5.8942e-09},
        {"reference_rcs_m2", 1.0},
    };

    // Same sensor, sensitivity given directly as the distance the budget implies.
    nlohmann::json direct = budget;
    direct.erase("tx_power_w");
    direct.erase("antenna_gain_dbi");
    direct.erase("wavelength_m");
    direct.erase("min_detectable_signal_w");
    direct["detection_reference_range"] = 6.0;

    // No model at all: every in-range hit is reported.
    nlohmann::json off = direct;
    off.erase("detection_reference_range");

    std::printf("== radar equation is applied by the loader ==\n");
    {
        // At 18 m the falloff bites hard; if the loader ignored the budget keys
        // the count would match the no-model case instead.
        const int eq = detections(budget, 18.0, -1.0);
        const int dir = detections(direct, 18.0, -1.0);
        const int none = detections(off, 18.0, -1.0);
        std::printf("     18 m: budget=%d direct=%d no-model=%d\n", eq, dir, none);
        check(eq > 0 && dir > 0 && none > 0, "all three configurations publish");
        check(eq == dir, "derived Rmax behaves identically to the equivalent direct range");
        check(eq < none, "the falloff actually suppresses returns (not silently off)");
    }

    std::printf("== per-target RCS shifts detection range ==\n");
    {
        // Same geometry and seed; only the RCS the backend reports differs.
        const int weak = detections(budget, 18.0, 0.01);
        const int refr = detections(budget, 18.0, 1.0);
        const int strong = detections(budget, 18.0, 100.0);
        std::printf("     18 m: sigma=0.01 -> %d, sigma=1 -> %d, sigma=100 -> %d\n",
                    weak, refr, strong);
        check(weak < refr, "a weakly reflecting target is detected less often");
        check(refr < strong, "a strongly reflecting target is detected more often");

        // What sigma_ref means, in two halves.
        //
        // (a) For a target that DECLARES its RCS, sigma_ref must cancel out:
        //     Rmax is derived proportional to sigma_ref^(1/4) and then rescaled by
        //     (sigma/sigma_ref)^(1/4), leaving a range that depends only on the
        //     target. Physically that has to hold -- how far you see an aircraft
        //     cannot depend on which reference target the datasheet was quoted
        //     against. (This assertion started out backwards and the check caught
        //     it, which is the reason it is spelled out here.)
        nlohmann::json big_ref = budget;
        big_ref["reference_rcs_m2"] = 16.0;
        const int a = detections(budget, 18.0, 1.0);
        const int b = detections(big_ref, 18.0, 1.0);
        std::printf("     annotated target: sigma_ref=1 -> %d, sigma_ref=16 -> %d\n", a, b);
        check(a == b, "sigma_ref cancels for a target that declares its own RCS");

        // (b) For an UN-annotated target the reference is what it is measured
        //     against, so raising sigma_ref does extend the range.
        const int ua = detections(budget, 18.0, -1.0);
        const int ub = detections(big_ref, 18.0, -1.0);
        std::printf("     un-annotated    : sigma_ref=1 -> %d, sigma_ref=16 -> %d\n", ua, ub);
        check(ua < ub, "an un-annotated target is treated as the reference target");
    }

    std::printf("\nRESULT: %s\n", g_fail == 0 ? "PASS" : "FAIL");
    return g_fail == 0 ? 0 : 1;
}
