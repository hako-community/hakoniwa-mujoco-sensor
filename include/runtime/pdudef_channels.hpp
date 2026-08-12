#pragma once

// Derive an SHM channel layout from a hakoniwa pdudef (issue #5).
//
// A bridge that publishes sensor PDUs has to know which SHM channel each PDU
// goes to. That mapping used to be hand-written on the launcher command line
// (A2_PDU_MAP="name=ch") in addition to the pdudef the master is started with,
// so every added sensor meant writing the same channel number twice with
// nothing to catch a disagreement -- and a wrong number does not fail loudly:
// the PDU simply goes somewhere else, or nowhere.
//
// The pdudef already states org_name, channel_id and pdu_size per robot, and
// the master is authoritative on it. So derive the mapping from that file
// instead of restating it. Declaring a sensor's channel once, in the file that
// has to declare it anyway, is then the whole job.
//
// Deliberately dependency-light -- nlohmann/json only, no MuJoCo and no PDU type
// headers -- so any bridge can include it and so the parsing is unit-testable
// without a running hakoniwa master.

#include <fstream>
#include <map>
#include <string>

#include <nlohmann/json.hpp>

namespace hako::robots::runtime
{
    // Where a PDU goes on the SHM: which channel, and how many bytes the channel
    // is declared to hold. pdu_size matters because a channel must be written
    // with exactly its declared width.
    struct ChannelSpec
    {
        int channel_id {-1};
        size_t pdu_size {0};
    };

    // org_name -> ChannelSpec for one robot in a pdudef.
    //
    // Both PDU lists are read. Readers first, then writers, so writers win where
    // a name appears in both: a bridge is a writer, and that is the entry whose
    // width it must honour. Consulting readers at all is what makes pdudefs that
    // declare a channel on only one side work.
    //
    // On any problem (missing file, bad JSON, unknown robot) the map comes back
    // empty and `err` is set -- the caller decides whether that is fatal.
    inline std::map<std::string, ChannelSpec> LoadPduDefChannels(
        const std::string& path, const std::string& robot, std::string& err)
    {
        std::map<std::string, ChannelSpec> out;
        err.clear();

        std::ifstream f(path);
        if (!f) {
            err = "cannot open " + path;
            return out;
        }

        nlohmann::json j;
        try {
            f >> j;
        } catch (const std::exception& e) {
            err = "parse error in " + path + ": " + e.what();
            return out;
        }

        if (!j.contains("robots") || !j["robots"].is_array()) {
            err = path + ": no \"robots\" array";
            return out;
        }

        for (const auto& r : j["robots"]) {
            if (!r.contains("name") || !r["name"].is_string()) continue;
            if (r["name"].get<std::string>() != robot) continue;

            for (const char* key : {"shm_pdu_readers", "shm_pdu_writers"}) {
                if (!r.contains(key) || !r[key].is_array()) continue;
                for (const auto& p : r[key]) {
                    if (!p.contains("org_name") || !p["org_name"].is_string()) continue;
                    if (!p.contains("channel_id") || !p["channel_id"].is_number_integer()) continue;
                    ChannelSpec spec {};
                    spec.channel_id = p["channel_id"].get<int>();
                    spec.pdu_size = p.value("pdu_size", 0);
                    out[p["org_name"].get<std::string>()] = spec;
                }
            }
            return out;
        }

        err = path + ": robot '" + robot + "' not found";
        return out;
    }
}
