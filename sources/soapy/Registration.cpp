#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Version.hpp>   
#include "MipiDevice.hpp"

static SoapySDR::KwargsList find(const SoapySDR::Kwargs &args)
{
    SoapySDR::Kwargs k;
    k["driver"] = "mipi";
    k["label"] = "QuadRF MIPI (CS8)";

    for (const auto &it : args)
    {
        // Don't overwrite our own driver tag and ignore client-only remote:* hints
        if (it.first == "driver") continue;
        if (it.first.rfind("remote", 0) == 0) continue;  
        k[it.first] = it.second;
    }
    return {k};
}

static SoapySDR::Device *make(const SoapySDR::Kwargs &args)
{
    return new MipiDevice(args);
}

// Correct signature: exactly 4 arguments.
static SoapySDR::Registry registerMipi("mipi", &find, &make, SOAPY_SDR_ABI_VERSION);
