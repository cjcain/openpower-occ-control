// Tests for P12 sensor D-Bus path construction in BuildTempDbusPaths().
//
// These tests exercise the path strings directly without requiring a live
// D-Bus, an OCC device, or a Status object.  Run them on any host (including
// Simics) with:
//
//   meson setup build -Dapp-poll-support=enabled -Dtests=enabled
//   meson test -C build --verbose "occ_sensor_path_test.cpp"

#include "occ_poll_handler.hpp"

#include <gtest/gtest.h>

using namespace open_power::occ;

// ---------------------------------------------------------------------------
// Minimal concrete subclass — only exposes BuildTempDbusPaths for testing.
// ---------------------------------------------------------------------------
class PathBuilder : public OccPollHandler
{
  public:
    // Bring the protected method into public scope.
    using OccPollHandler::BuildTempDbusPaths;

    // Re-export the protected fruType enum values as public constants so
    // test bodies can reference them without a protected-access error.
    static constexpr auto k_processorCore = OccPollHandler::processorCore;
    static constexpr auto k_processorMMA = OccPollHandler::processorMMA;
    static constexpr auto k_VRMVdd = OccPollHandler::VRMVdd;
    static constexpr auto k_processorIoRing = OccPollHandler::processorIoRing;
    static constexpr auto k_dimm = OccPollHandler::dimm;
    static constexpr auto k_internalMemCtlr = OccPollHandler::internalMemCtlr;
    static constexpr auto k_memCtrlAndDimm = OccPollHandler::memCtrlAndDimm;
    static constexpr auto k_PMIC = OccPollHandler::PMIC;
    static constexpr auto k_memCtlrExSensor = OccPollHandler::memCtlrExSensor;
    static constexpr auto k_FRU_UNAVAILABLE = OccPollHandler::FRU_UNAVAILABLE;

    // Expose the protected power sensor name map for path tests.
    const std::map<std::string, std::string>& getPowerSensorName() const
    {
        return powerSensorName;
    }

    // Pure-virtual stubs — not exercised by path tests.
    void HandlePollAction() override {}
    bool pollReadStateStatus(unsigned int&, int&) override
    {
        return false;
    }
    bool pollReadPcapBounds(uint32_t&, uint32_t&, uint32_t&) override
    {
        return false;
    }
};

// Suppress the unused-function warning on utils.hpp's getBus() static which
// is dragged in transitively but not needed by this test.
namespace
{
[[maybe_unused]] void suppress_getBus_warning()
{
    open_power::occ::utils::getBus();
}
} // namespace

// ---------------------------------------------------------------------------
// Helper: call BuildTempDbusPaths, print the resulting paths, and return them.
// ---------------------------------------------------------------------------
struct SensorPaths
{
    std::string sensor;
    std::string dvfs;
    std::string tcontrol;
};

static SensorPaths buildPaths(uint32_t sensorID, uint32_t fruType,
                              uint32_t occInstance, bool isHottest = false)
{
    PathBuilder pb;
    std::string sensor, dvfs, tcontrol;
    pb.BuildTempDbusPaths(sensor, dvfs, tcontrol, sensorID, fruType,
                          occInstance, isHottest);
    std::cout << "  sensor:   " << sensor << "\n";
    if (!dvfs.empty())
    {
        std::cout << "  dvfs:     " << dvfs << "\n";
    }
    if (!tcontrol.empty())
    {
        std::cout << "  tcontrol: " << tcontrol << "\n";
    }
    return {sensor, dvfs, tcontrol};
}

// Convenience: pack a SensorID with the given type byte and instanceID.
// SensorID layout: [type(8) | 0x00(8) | instanceID(16)]
static constexpr uint32_t makeID(uint8_t type, uint16_t instanceID)
{
    return (static_cast<uint32_t>(type) << 24) | instanceID;
}

static const std::string TROOT = "/xyz/openbmc_project/sensors/temperature/";

// ===========================================================================
// VRMVdd — no dvfs, no tcontrol
// ===========================================================================
TEST(SensorPath, VRMVdd_proc0)
{
    auto [s, d, t] = buildPaths(0, PathBuilder::k_VRMVdd, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_vrm_vdd_temp");
    EXPECT_EQ(d, "");
    EXPECT_EQ(t, "");
}

TEST(SensorPath, VRMVdd_proc1)
{
    auto [s, d, t] = buildPaths(0, PathBuilder::k_VRMVdd, 1);
    EXPECT_EQ(s, TROOT + "chassis_proc1_vrm_vdd_temp");
    EXPECT_EQ(t, "");
}

// ===========================================================================
// ProcessorIoRing — dvfs but no tcontrol
// ===========================================================================
TEST(SensorPath, IoRing_proc0)
{
    auto [s, d, t] = buildPaths(0, PathBuilder::k_processorIoRing, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_ioring_temp");
    EXPECT_EQ(d, TROOT + "chassis_proc0_ioring_dvfs_temp");
    EXPECT_EQ(t, "");
}

TEST(SensorPath, IoRing_proc3)
{
    auto [s, d, t] = buildPaths(0, PathBuilder::k_processorIoRing, 3);
    EXPECT_EQ(s, TROOT + "chassis_proc3_ioring_temp");
    EXPECT_EQ(d, TROOT + "chassis_proc3_ioring_dvfs_temp");
    EXPECT_EQ(t, "");
}

// ===========================================================================
// Core temp — dvfs AND tcontrol, both scoped to the tap group
//
// Per-core path:  chassis_procY_coregroupZ_coreW_temp
//   Z = tapGroup  = instanceID / 8  (0-7)
//   W = coreInTap = instanceID % 8  (0-7)
//
// Tap-level path (isHottest): chassis_procY_coregroupZ_temp
//
// dvfs:     chassis_procY_coregroupZ_dvfs_temp
// tcontrol: chassis_procY_coregroupZ_tcontrol_temp
//
// Tap layout (per proc, 64 small cores total):
//   tap0 = cores  0- 7   instanceID  0- 7
//   tap1 = cores  8-15   instanceID  8-15
//   tap2 = cores 16-23   instanceID 16-23
//   tap3 = cores 24-31   instanceID 24-31
//   tap4 = cores 32-39   instanceID 32-39
//   tap5 = cores 40-47   instanceID 40-47
//   tap6 = cores 48-55   instanceID 48-55
//   tap7 = cores 56-63   instanceID 56-63
// ===========================================================================

// --- tap0: verify coreInTap ---
TEST(SensorPath, Core_tap0_instanceID0_proc0)
{
    // instanceID=0 → core=0, tap=0, coreInTap=0
    auto [s, d, t] =
        buildPaths(makeID(SID_TYPE_CORE, 0), PathBuilder::k_processorCore, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_coregroup0_core0_temp");
    EXPECT_EQ(d, TROOT + "chassis_proc0_coregroup0_dvfs_temp");
    EXPECT_EQ(t, TROOT + "chassis_proc0_coregroup0_tcontrol_temp");
}

TEST(SensorPath, Core_tap0_instanceID1_proc0)
{
    // instanceID=1 → tap=0, coreInTap=1
    auto [s, d, t] =
        buildPaths(makeID(SID_TYPE_CORE, 1), PathBuilder::k_processorCore, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_coregroup0_core1_temp");
    EXPECT_EQ(d, TROOT + "chassis_proc0_coregroup0_dvfs_temp");
    EXPECT_EQ(t, TROOT + "chassis_proc0_coregroup0_tcontrol_temp");
}

TEST(SensorPath, Core_tap0_instanceID6_proc0)
{
    // instanceID=6 → tap=0, coreInTap=6
    auto [s, d, t] =
        buildPaths(makeID(SID_TYPE_CORE, 6), PathBuilder::k_processorCore, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_coregroup0_core6_temp");
    EXPECT_EQ(t, TROOT + "chassis_proc0_coregroup0_tcontrol_temp");
}

TEST(SensorPath, Core_tap0_instanceID7_proc0)
{
    // instanceID=7 → tap=0, coreInTap=7
    auto [s, d, t] =
        buildPaths(makeID(SID_TYPE_CORE, 7), PathBuilder::k_processorCore, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_coregroup0_core7_temp");
    EXPECT_EQ(t, TROOT + "chassis_proc0_coregroup0_tcontrol_temp");
}

// --- tap1: boundary at instanceID=8 ---
TEST(SensorPath, Core_tap1_instanceID8_proc0)
{
    // instanceID=8 → core=8, tap=1, coreInTap=0
    auto [s, d, t] =
        buildPaths(makeID(SID_TYPE_CORE, 8), PathBuilder::k_processorCore, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_coregroup1_core0_temp");
    EXPECT_EQ(d, TROOT + "chassis_proc0_coregroup1_dvfs_temp");
    EXPECT_EQ(t, TROOT + "chassis_proc0_coregroup1_tcontrol_temp");
}

TEST(SensorPath, Core_tap1_instanceID15_proc0)
{
    // instanceID=15 → tap=1, coreInTap=7
    auto [s, d, t] =
        buildPaths(makeID(SID_TYPE_CORE, 15), PathBuilder::k_processorCore, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_coregroup1_core7_temp");
    EXPECT_EQ(t, TROOT + "chassis_proc0_coregroup1_tcontrol_temp");
}

// --- tap2 ---
TEST(SensorPath, Core_tap2_instanceID16_proc0)
{
    // instanceID=16 → core=16, tap=2, coreInTap=0
    auto [s, d, t] =
        buildPaths(makeID(SID_TYPE_CORE, 16), PathBuilder::k_processorCore, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_coregroup2_core0_temp");
    EXPECT_EQ(d, TROOT + "chassis_proc0_coregroup2_dvfs_temp");
    EXPECT_EQ(t, TROOT + "chassis_proc0_coregroup2_tcontrol_temp");
}

// --- tap3 ---
TEST(SensorPath, Core_tap3_instanceID24_proc0)
{
    // instanceID=24 → core=24, tap=3, coreInTap=0
    auto [s, d, t] =
        buildPaths(makeID(SID_TYPE_CORE, 24), PathBuilder::k_processorCore, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_coregroup3_core0_temp");
    EXPECT_EQ(d, TROOT + "chassis_proc0_coregroup3_dvfs_temp");
    EXPECT_EQ(t, TROOT + "chassis_proc0_coregroup3_tcontrol_temp");
}

// --- tap4 ---
TEST(SensorPath, Core_tap4_instanceID32_proc0)
{
    // instanceID=32 → core=32, tap=4, coreInTap=0
    auto [s, d, t] =
        buildPaths(makeID(SID_TYPE_CORE, 32), PathBuilder::k_processorCore, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_coregroup4_core0_temp");
    EXPECT_EQ(d, TROOT + "chassis_proc0_coregroup4_dvfs_temp");
    EXPECT_EQ(t, TROOT + "chassis_proc0_coregroup4_tcontrol_temp");
}

// --- tap5 ---
TEST(SensorPath, Core_tap5_instanceID40_proc0)
{
    // instanceID=40 → core=40, tap=5, coreInTap=0
    auto [s, d, t] =
        buildPaths(makeID(SID_TYPE_CORE, 40), PathBuilder::k_processorCore, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_coregroup5_core0_temp");
    EXPECT_EQ(d, TROOT + "chassis_proc0_coregroup5_dvfs_temp");
    EXPECT_EQ(t, TROOT + "chassis_proc0_coregroup5_tcontrol_temp");
}

// --- tap6 ---
TEST(SensorPath, Core_tap6_instanceID48_proc0)
{
    // instanceID=48 → core=48, tap=6, coreInTap=0
    auto [s, d, t] =
        buildPaths(makeID(SID_TYPE_CORE, 48), PathBuilder::k_processorCore, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_coregroup6_core0_temp");
    EXPECT_EQ(d, TROOT + "chassis_proc0_coregroup6_dvfs_temp");
    EXPECT_EQ(t, TROOT + "chassis_proc0_coregroup6_tcontrol_temp");
}

// --- tap7 ---
TEST(SensorPath, Core_tap7_instanceID56_proc0)
{
    // instanceID=56 → core=56, tap=7, coreInTap=0
    auto [s, d, t] =
        buildPaths(makeID(SID_TYPE_CORE, 56), PathBuilder::k_processorCore, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_coregroup7_core0_temp");
    EXPECT_EQ(d, TROOT + "chassis_proc0_coregroup7_dvfs_temp");
    EXPECT_EQ(t, TROOT + "chassis_proc0_coregroup7_tcontrol_temp");
}

TEST(SensorPath, Core_tap7_instanceID63_proc0)
{
    // instanceID=63 → tap=7, coreInTap=7
    auto [s, d, t] =
        buildPaths(makeID(SID_TYPE_CORE, 63), PathBuilder::k_processorCore, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_coregroup7_core7_temp");
    EXPECT_EQ(t, TROOT + "chassis_proc0_coregroup7_tcontrol_temp");
}

// --- tap-level (isHottest=true): OCC-determined tap temperature ---
TEST(SensorPath, Core_tap_level_tap0_proc0)
{
    // instanceID=0 → tapGroup=0; isHottest publishes the tap-level sensor
    auto [s, d, t] = buildPaths(makeID(SID_TYPE_CORE, 0),
                                PathBuilder::k_processorCore, 0, true);
    EXPECT_EQ(s, TROOT + "chassis_proc0_coregroup0_temp");
    EXPECT_EQ(d, TROOT + "chassis_proc0_coregroup0_dvfs_temp");
    EXPECT_EQ(t, TROOT + "chassis_proc0_coregroup0_tcontrol_temp");
}

TEST(SensorPath, Core_tap_level_tap3_proc0)
{
    // instanceID=24 → tapGroup=3
    auto [s, d, t] = buildPaths(makeID(SID_TYPE_CORE, 24),
                                PathBuilder::k_processorCore, 0, true);
    EXPECT_EQ(s, TROOT + "chassis_proc0_coregroup3_temp");
    EXPECT_EQ(d, TROOT + "chassis_proc0_coregroup3_dvfs_temp");
    EXPECT_EQ(t, TROOT + "chassis_proc0_coregroup3_tcontrol_temp");
}

// --- multi-proc ---
TEST(SensorPath, Core_tap3_instanceID24_proc2)
{
    // instanceID=24 → core=24, tap=3, coreInTap=0 on proc2
    auto [s, d, t] =
        buildPaths(makeID(SID_TYPE_CORE, 24), PathBuilder::k_processorCore, 2);
    EXPECT_EQ(s, TROOT + "chassis_proc2_coregroup3_core0_temp");
    EXPECT_EQ(d, TROOT + "chassis_proc2_coregroup3_dvfs_temp");
    EXPECT_EQ(t, TROOT + "chassis_proc2_coregroup3_tcontrol_temp");
}

// ===========================================================================
// MMA temp — tap-level only: instanceID = 0xFF00 | tapGroup
// Per-core MMA instanceIDs (0-63) return false (not published).
// ===========================================================================

// --- non-sentinel instanceID returns false ---
TEST(SensorPath, MMA_per_core_not_published)
{
    PathBuilder pb;
    std::string s, d, t;
    bool found = pb.BuildTempDbusPaths(s, d, t, makeID(SID_TYPE_CORE, 0),
                                       PathBuilder::k_processorMMA, 0);
    EXPECT_FALSE(found);
}

// --- tap-level MMA: instanceID = 0xFF00 | tapGroup ---
TEST(SensorPath, MMA_tap_level_tap0_proc0)
{
    // instanceID=0xFF00 → tap-level sentinel for tap0
    // MMA has tcontrol but no dvfs.
    auto [s, d, t] = buildPaths(makeID(SID_TYPE_CORE, 0xFF00),
                                PathBuilder::k_processorMMA, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_coregroup0_mma_temp");
    EXPECT_EQ(d, "");
    EXPECT_EQ(t, TROOT + "chassis_proc0_coregroup0_mma_tcontrol_temp");
}

TEST(SensorPath, MMA_tap_level_tap3_proc0)
{
    // instanceID=0xFF03 → tap-level sentinel for tap3
    auto [s, d, t] = buildPaths(makeID(SID_TYPE_CORE, 0xFF03),
                                PathBuilder::k_processorMMA, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_coregroup3_mma_temp");
    EXPECT_EQ(d, "");
    EXPECT_EQ(t, TROOT + "chassis_proc0_coregroup3_mma_tcontrol_temp");
}

TEST(SensorPath, MMA_tap_level_tap7_proc1)
{
    // instanceID=0xFF07 → tap-level sentinel for tap7 on proc1
    auto [s, d, t] = buildPaths(makeID(SID_TYPE_CORE, 0xFF07),
                                PathBuilder::k_processorMMA, 1);
    EXPECT_EQ(s, TROOT + "chassis_proc1_coregroup7_mma_temp");
    EXPECT_EQ(d, "");
    EXPECT_EQ(t, TROOT + "chassis_proc1_coregroup7_mma_tcontrol_temp");
}

// ===========================================================================
// DIMM temp — all five fruTypes; no tcontrol for DIMM sensors
// ===========================================================================
TEST(SensorPath, DIMM_dram_instance5_proc0)
{
    auto [s, d,
          t] = buildPaths(makeID(SID_TYPE_DIMM, 5), PathBuilder::k_dimm, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_dimm5_dram_temp");
    EXPECT_EQ(d, TROOT + "chassis_proc0_dimm_dram_dvfs_temp");
    EXPECT_EQ(t, "");
}

TEST(SensorPath, DIMM_intmb_instance2_proc0)
{
    auto [s, d, t] =
        buildPaths(makeID(SID_TYPE_DIMM, 2), PathBuilder::k_internalMemCtlr, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_dimm2_intmb_temp");
    EXPECT_EQ(d, TROOT + "chassis_proc0_dimm_intmb_dvfs_temp");
    EXPECT_EQ(t, "");
}

TEST(SensorPath, DIMM_dram_extmb_instance0_proc0)
{
    auto [s, d, t] =
        buildPaths(makeID(SID_TYPE_DIMM, 0), PathBuilder::k_memCtrlAndDimm, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_dimm0_dram_extmb_temp");
    EXPECT_EQ(d, TROOT + "chassis_proc0_dimm_dram_extmb_dvfs_temp");
    EXPECT_EQ(t, "");
}

TEST(SensorPath, DIMM_pmic_instance1_proc0)
{
    auto [s, d,
          t] = buildPaths(makeID(SID_TYPE_DIMM, 1), PathBuilder::k_PMIC, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_dimm1_pmic_temp");
    EXPECT_EQ(d, TROOT + "chassis_proc0_dimm_pmic_dvfs_temp");
    EXPECT_EQ(t, "");
}

TEST(SensorPath, DIMM_extmb_instance3_proc0)
{
    auto [s, d, t] =
        buildPaths(makeID(SID_TYPE_DIMM, 3), PathBuilder::k_memCtlrExSensor, 0);
    EXPECT_EQ(s, TROOT + "chassis_proc0_dimm3_extmb_temp");
    EXPECT_EQ(d, TROOT + "chassis_proc0_dimm_extmb_dvfs_temp");
    EXPECT_EQ(t, "");
}

// --- hottest variants ---
TEST(SensorPath, DIMM_dram_hottest_proc0)
{
    auto [s, d, t] =
        buildPaths(makeID(SID_TYPE_DIMM, 0), PathBuilder::k_dimm, 0, true);
    EXPECT_EQ(s, TROOT + "chassis_proc0_dimm_dram_temp_hottest");
    EXPECT_EQ(t, "");
}

TEST(SensorPath, DIMM_pmic_hottest_proc0)
{
    auto [s, d, t] =
        buildPaths(makeID(SID_TYPE_DIMM, 0), PathBuilder::k_PMIC, 0, true);
    EXPECT_EQ(s, TROOT + "chassis_proc0_dimm_pmic_temp_hottest");
    EXPECT_EQ(t, "");
}

// --- unavailable ---
TEST(SensorPath, DIMM_unavailable_returns_false)
{
    PathBuilder pb;
    std::string s, d, t;
    bool found = pb.BuildTempDbusPaths(s, d, t, makeID(SID_TYPE_DIMM, 0),
                                       PathBuilder::k_FRU_UNAVAILABLE, 0);
    EXPECT_FALSE(found);
}

// ===========================================================================
// Power path tests
//
// Power paths are built as:
//   proc-scoped:   /xyz/openbmc_project/sensors/power/chassis_procY_<leaf>
//   system-wide:   /xyz/openbmc_project/sensors/power/<leaf>  (no proc prefix)
//   chiplet
//   (EXTN):/xyz/openbmc_project/sensors/power/chassis_procY_chiplet_power
//                  /xyz/openbmc_project/sensors/power/chassis_procY_chiplet_mem_power
// ===========================================================================

static const std::string PROOT = "/xyz/openbmc_project/sensors/power/";

// Helper: look up functionID in powerSensorName, print and return the full
// path.
static std::string buildPowerPath(const std::string& functionID,
                                  uint32_t occInstance)
{
    PathBuilder pb;
    const auto& map = pb.getPowerSensorName();
    auto iter = map.find(functionID);
    if (iter == map.end())
    {
        std::cout << "  power[" << functionID << "]: <not found>\n";
        return "";
    }
    std::string path;
    path = PROOT + "chassis_proc" + std::to_string(occInstance) + "_" +
           iter->second;
    std::cout << "  power[" << functionID << "]: " << path << "\n";
    return path;
}

// --- system-wide ---
TEST(PowerPath, TotalPower_system)
{
    EXPECT_EQ(buildPowerPath("system", 0), PROOT + "chassis_proc0_total_power");
}

// --- memory ---
TEST(PowerPath, SledMem_proc0)
{
    EXPECT_EQ(buildPowerPath("1", 0), PROOT + "chassis_proc0_p0_mem_power");
    EXPECT_EQ(buildPowerPath("2", 0), PROOT + "chassis_proc0_p1_mem_power");
    EXPECT_EQ(buildPowerPath("3", 0), PROOT + "chassis_proc0_p2_mem_power");
    EXPECT_EQ(buildPowerPath("4", 0), PROOT + "chassis_proc0_p3_mem_power");
}

// --- processor ---
TEST(PowerPath, SledProc_proc1)
{
    EXPECT_EQ(buildPowerPath("5", 1), PROOT + "chassis_proc1_p0_power");
    EXPECT_EQ(buildPowerPath("6", 1), PROOT + "chassis_proc1_p1_power");
    EXPECT_EQ(buildPowerPath("7", 1), PROOT + "chassis_proc1_p2_power");
    EXPECT_EQ(buildPowerPath("8", 1), PROOT + "chassis_proc1_p3_power");
}

// --- cache ---
TEST(PowerPath, SledCache_proc0)
{
    EXPECT_EQ(buildPowerPath("9", 0), PROOT + "chassis_proc0_p0_cache_power");
    EXPECT_EQ(buildPowerPath("10", 0), PROOT + "chassis_proc0_p1_cache_power");
    EXPECT_EQ(buildPowerPath("11", 0), PROOT + "chassis_proc0_p2_cache_power");
    EXPECT_EQ(buildPowerPath("12", 0), PROOT + "chassis_proc0_p3_cache_power");
}

// --- IO ---
TEST(PowerPath, IO_proc0)
{
    EXPECT_EQ(buildPowerPath("13", 0), PROOT + "chassis_proc0_io_a_power");
    EXPECT_EQ(buildPowerPath("14", 0), PROOT + "chassis_proc0_io_b_power");
    EXPECT_EQ(buildPowerPath("15", 0), PROOT + "chassis_proc0_io_c_power");
}

// --- fans ---
TEST(PowerPath, Fans_proc0)
{
    EXPECT_EQ(buildPowerPath("16", 0), PROOT + "chassis_proc0_fans_a_power");
    EXPECT_EQ(buildPowerPath("17", 0), PROOT + "chassis_proc0_fans_b_power");
}

// --- storage ---
TEST(PowerPath, Storage_proc0)
{
    EXPECT_EQ(buildPowerPath("18", 0), PROOT + "chassis_proc0_storage_a_power");
    EXPECT_EQ(buildPowerPath("19", 0), PROOT + "chassis_proc0_storage_b_power");
}

// --- memory cache ---
TEST(PowerPath, MemCache_proc0)
{
    EXPECT_EQ(buildPowerPath("23", 0), PROOT + "chassis_proc0_mem_cache_power");
}

// --- memory channel ---
TEST(PowerPath, SledMemChan_proc0)
{
    EXPECT_EQ(buildPowerPath("25", 0), PROOT + "chassis_proc0_p0_mem_0_power");
    EXPECT_EQ(buildPowerPath("26", 0), PROOT + "chassis_proc0_p0_mem_1_power");
    EXPECT_EQ(buildPowerPath("27", 0), PROOT + "chassis_proc0_p0_mem_2_power");
}

// --- PCIe ---
TEST(PowerPath, PCIe_proc0)
{
    EXPECT_EQ(buildPowerPath("34", 0), PROOT + "chassis_proc0_pcie_power");
    EXPECT_EQ(buildPowerPath("35", 0), PROOT + "chassis_proc0_pcie_dcm0_power");
    EXPECT_EQ(buildPowerPath("36", 0), PROOT + "chassis_proc0_pcie_dcm1_power");
    EXPECT_EQ(buildPowerPath("37", 0), PROOT + "chassis_proc0_pcie_dcm2_power");
    EXPECT_EQ(buildPowerPath("38", 0), PROOT + "chassis_proc0_pcie_dcm3_power");
}

// --- IO DCM ---
TEST(PowerPath, IO_DCM_proc0)
{
    EXPECT_EQ(buildPowerPath("39", 0), PROOT + "chassis_proc0_io_dcm0_power");
    EXPECT_EQ(buildPowerPath("40", 0), PROOT + "chassis_proc0_io_dcm1_power");
    EXPECT_EQ(buildPowerPath("41", 0), PROOT + "chassis_proc0_io_dcm2_power");
    EXPECT_EQ(buildPowerPath("42", 0), PROOT + "chassis_proc0_io_dcm3_power");
}

// --- AVDD ---
TEST(PowerPath, AVDD_proc0)
{
    EXPECT_EQ(buildPowerPath("43", 0),
              PROOT + "chassis_proc0_avdd_total_power");
}

// --- unknown function ID returns empty ---
TEST(PowerPath, UnknownFunctionID)
{
    EXPECT_EQ(buildPowerPath("99", 0), "");
}

// --- chiplet power (EXTN — inline paths, not from map) ---
TEST(PowerPath, Chiplet_proc0)
{
    std::string p = PROOT + "chassis_proc0_chiplet_power";
    std::string m = PROOT + "chassis_proc0_chiplet_mem_power";
    std::cout << "  chiplet:     " << p << "\n";
    std::cout << "  chiplet_mem: " << m << "\n";
    EXPECT_EQ(PROOT + "chassis_proc0_" + "chiplet_power", p);
    EXPECT_EQ(PROOT + "chassis_proc0_" + "chiplet_mem_power", m);
}

TEST(PowerPath, Chiplet_proc1)
{
    std::string p = PROOT + "chassis_proc1_chiplet_power";
    std::string m = PROOT + "chassis_proc1_chiplet_mem_power";
    std::cout << "  chiplet:     " << p << "\n";
    std::cout << "  chiplet_mem: " << m << "\n";
    EXPECT_EQ(PROOT + "chassis_proc1_" + "chiplet_power", p);
    EXPECT_EQ(PROOT + "chassis_proc1_" + "chiplet_mem_power", m);
}
