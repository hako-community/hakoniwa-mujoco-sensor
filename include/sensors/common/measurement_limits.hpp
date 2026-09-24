#pragma once

#include <algorithm>

namespace hako::robots::sensor::common
{
    struct LimitedScalar
    {
        double value {0.0};
        bool saturated {false};
    };

    // ADC/output-range stage.  Callers OR SensorStatus::Saturated into their
    // envelope when saturated is true; clipping never silently looks like truth.
    inline LimitedScalar ClampMeasurement(double value, double minimum, double maximum)
    {
        const double clamped = std::clamp(value, minimum, maximum);
        return LimitedScalar {clamped, clamped != value};
    }
}
