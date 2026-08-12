// Issue #5: derive the SHM channel layout from the pdudef instead of restating
// it on the launcher command line.
//
// Adding a third radar used to mean writing its channel number twice -- once in
// the pdudef the master is started with, once in A2_PDU_MAP -- and a
// disagreement between the two does not fail loudly: the PDU goes to the wrong
// channel, or to none. These cases pin the parsing that removes the second copy.
//
// Dependency-light by design: no MuJoCo, no PDU types, no running master.

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "runtime/pdudef_channels.hpp"

namespace rt = hako::robots::runtime;

static int g_fail = 0;
static int g_checks = 0;
static std::vector<std::string> g_temp_files;

static void check(bool cond, const char* msg)
{
    ++g_checks;
    if (!cond) { ++g_fail; std::printf("  [FAIL] %s\n", msg); }
    else { std::printf("  [ ok ] %s\n", msg); }
}

static std::string write_temp(const char* name, const std::string& body)
{
    const std::string path = std::string("pdudef_test_") + name + ".json";
    std::ofstream f(path);
    f << body;
    g_temp_files.push_back(path);
    return path;
}

// Shaped like the real config2/webavatar-2-radar2.json: two robots, the sensor
// channels declared on BOTH the reader and writer side.
static const char* kTwoRobots = R"({
  "robots": [
    {
      "name": "Drone",
      "shm_pdu_readers": [
        {"org_name": "pos",               "channel_id": 1,  "pdu_size": 72},
        {"org_name": "lidar_points",      "channel_id": 16, "pdu_size": 177424},
        {"org_name": "radar_points",      "channel_id": 19, "pdu_size": 177424},
        {"org_name": "radar_points_rear", "channel_id": 21, "pdu_size": 177424}
      ],
      "shm_pdu_writers": [
        {"org_name": "lidar_points",      "channel_id": 16, "pdu_size": 177424},
        {"org_name": "radar_points",      "channel_id": 19, "pdu_size": 177424},
        {"org_name": "radar_points_rear", "channel_id": 21, "pdu_size": 177424}
      ]
    },
    {
      "name": "Drone1",
      "shm_pdu_writers": [
        {"org_name": "radar_points", "channel_id": 31, "pdu_size": 4096}
      ]
    }
  ]
})";

int main()
{
    std::printf("== the layout comes out of the pdudef ==\n");
    {
        const auto path = write_temp("two_robots", kTwoRobots);
        std::string err;
        const auto m = rt::LoadPduDefChannels(path, "Drone", err);

        check(err.empty(), "a well-formed pdudef parses without error");
        check(m.size() == 4, "every declared channel is picked up");
        check(m.count("radar_points_rear") == 1,
              "the THIRD radar needs no hand-written mapping -- this is #5");
        check(m.at("radar_points_rear").channel_id == 21, "its channel id comes from the file");
        check(m.at("radar_points_rear").pdu_size == 177424, "so does its declared width");
        check(m.at("pos").channel_id == 1 && m.at("pos").pdu_size == 72,
              "non-sensor channels are read too, at their own size");
    }

    std::printf("\n== the robot selects the layout ==\n");
    {
        const auto path = write_temp("two_robots", kTwoRobots);
        std::string err;
        const auto m = rt::LoadPduDefChannels(path, "Drone1", err);

        check(err.empty(), "the second robot parses");
        // Same org_name, different channel: two aircraft may carry different
        // fits, so reading the wrong robot's block silently misroutes the PDU.
        check(m.size() == 1 && m.at("radar_points").channel_id == 31,
              "each robot gets ITS OWN channel for the same org_name");
        check(m.count("lidar_points") == 0, "another robot's channels do not leak in");
    }

    std::printf("\n== writers win where a name appears on both sides ==\n");
    {
        // A bridge writes; the width it must honour is the writer's.
        const auto path = write_temp("conflict", R"({
          "robots": [{
            "name": "Drone",
            "shm_pdu_readers": [{"org_name": "radar_points", "channel_id": 19, "pdu_size": 8}],
            "shm_pdu_writers": [{"org_name": "radar_points", "channel_id": 19, "pdu_size": 177424}]
          }]
        })");
        std::string err;
        const auto m = rt::LoadPduDefChannels(path, "Drone", err);
        check(m.at("radar_points").pdu_size == 177424, "the writer entry is the one kept");
    }

    std::printf("\n== a reader-only declaration still maps ==\n");
    {
        const auto path = write_temp("reader_only", R"({
          "robots": [{
            "name": "Drone",
            "shm_pdu_readers": [{"org_name": "radar_points", "channel_id": 19, "pdu_size": 177424}]
          }]
        })");
        std::string err;
        const auto m = rt::LoadPduDefChannels(path, "Drone", err);
        check(err.empty() && m.at("radar_points").channel_id == 19,
              "pdudefs that declare only one side are usable");
    }

    std::printf("\n== bad input reports, and does not pretend ==\n");
    {
        std::string err;

        const auto missing = rt::LoadPduDefChannels("no_such_file_here.json", "Drone", err);
        check(missing.empty() && !err.empty(), "a missing file is an error, not an empty success");

        const auto bad = rt::LoadPduDefChannels(write_temp("bad", "{ not json"), "Drone", err);
        check(bad.empty() && !err.empty(), "malformed JSON is reported");

        const auto noarr = rt::LoadPduDefChannels(write_temp("noarr", R"({"x":1})"), "Drone", err);
        check(noarr.empty() && !err.empty(), "a file with no robots array is reported");

        const auto absent = rt::LoadPduDefChannels(write_temp("two_robots2", kTwoRobots),
                                                   "Nonexistent", err);
        check(absent.empty() && !err.empty(),
              "an unknown robot is reported -- silence here would mean publishing nowhere");

        // Entries missing the fields we need are skipped, not guessed at.
        const auto partial = rt::LoadPduDefChannels(write_temp("partial", R"({
          "robots": [{
            "name": "Drone",
            "shm_pdu_writers": [
              {"org_name": "no_channel"},
              {"channel_id": 5},
              {"org_name": "ok", "channel_id": 7}
            ]
          }]
        })"), "Drone", err);
        check(err.empty() && partial.size() == 1 && partial.count("ok") == 1,
              "incomplete entries are skipped, complete ones still land");
        check(partial.at("ok").pdu_size == 0,
              "a missing pdu_size reads as 0 so the caller can fall back");
    }

    // Leave no scratch JSON behind in the build dir.
    for (const auto& p : g_temp_files) std::remove(p.c_str());

    std::printf("\n%d/%d checks passed\n", g_checks - g_fail, g_checks);
    if (g_fail > 0) {
        std::printf("RESULT: FAIL (%d failures)\n", g_fail);
        return 1;
    }
    std::printf("RESULT: PASS\n");
    return 0;
}
