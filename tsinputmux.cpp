#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <errno.h>
#include <getopt.h>
#include <ctype.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>

#include <netinet/in.h> /* IPPROTO_IP, sockaddr_in, htons(),
htonl() */
#include <arpa/inet.h>  /* inet_addr() */
#include <netdb.h>
#include <time.h>
#include <sys/ioctl.h>
#include <queue>
#include "ts_util/pcrpts.h"
#include "./dvbs2neon/dvbs2neon.h"
#include "./dvbsarm/fec100.h"

extern "C"
{
    extern void dvbsenco_init(void);
    extern uchar *dvbsenco(uchar *);
}

using namespace std;
#define WITH_NEON
//#define UDP_BUFF_MAX_BBFRAME (58192 / 8)
#define UDP_BUFF_MAX_BBFRAME 8000

void correctcc(uint8_t *tspacket, bool discountinuity);
extern int m_Fecmode;
enum
{
    fec_fix,
    fec_variable
};

typedef struct
{
    ssize_t size;
    ssize_t modecod;
    uint8_t bbframe[UDP_BUFF_MAX_BBFRAME];
} buffer_t;

extern queue<buffer_t *> m_bbframe_queue;
extern pthread_mutex_t buffer_mutextx;
extern int m_txmode;
extern uint32_t m_efficiency;
extern int m_FecRange;

struct bbheader
{
    uint8_t matype1;
    uint8_t matype2;
    uint16_t upl;
    uint16_t dfl;
    uint8_t sync;
    uint8_t syncd1;
    uint8_t syncd2;
    uint8_t crc;
};

uchar BBFrameNeonBuff[144000] __attribute__((aligned(128)));
uchar symbolbuff[144 * 1024] __attribute__((aligned(16)));

static DVB2FrameFormat fmt;
enum
{
    longframe,
    shortframe

};
enum
{
    mod_qpsk,
    mod_8psk,
    mod_16apsk,
    mod_32apsk
};
enum
{
    C1_4,
    C1_3,
    C2_5,
    C1_2,
    C3_5,
    C2_3,
    C3_4,
    C4_5,
    C5_6,
    C8_9,
    C9_10
};
int m_ModeCod = C2_3 + longframe;
uint8_t m_variable_ts_coderate;
#define MAX_QUEUE_ITEM 100
#define MAX_QUEUE_CHANGEMODCOD 2

// =============================================================================
//  BitrateEstimator — sliding window input bitrate measurement
//
//  addBytes() is called for every real (non-null) TS packet entering addneonts().
//  getRateKbps() returns the average input bitrate over the last WINDOW_MS ms.
//  This gives the adaptive controller a direct measurement of what the input
//  actually needs, independent of queue depth.
// =============================================================================
#define BITRATE_WINDOW_MS  1000   // measurement window in ms

class BitrateEstimator
{
public:
    BitrateEstimator() : m_head(0), m_totalBytes(0) {}

    void addBytes(size_t n)
    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        uint64_t nowMs = (uint64_t)now.tv_sec * 1000ULL + now.tv_nsec / 1000000ULL;

        // Expire old buckets outside the window
        expireOld(nowMs);

        // Add to current bucket (create if needed)
        if (m_head == 0 || m_buckets[m_head - 1].timeMs != nowMs)
        {
            if (m_head < MAX_BUCKETS)
            {
                m_buckets[m_head].timeMs = nowMs;
                m_buckets[m_head].bytes  = n;
                m_head++;
            }
        }
        else
        {
            m_buckets[m_head - 1].bytes += n;
        }
        m_totalBytes += n;
    }

    // Returns bitrate in kbps based on the current window
    uint32_t getRateKbps() const
    {
        if (m_totalBytes == 0 || m_head == 0) return 0;
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        uint64_t nowMs = (uint64_t)now.tv_sec * 1000ULL + now.tv_nsec / 1000000ULL;
        uint64_t oldest = (m_head > 0) ? m_buckets[0].timeMs : nowMs;
        uint64_t spanMs = nowMs - oldest;
        // Always divide by at least the full window width. Using a 50ms floor
        // caused 5-10x overestimation when the UDP socket buffer delivered many
        // packets in a single burst: all bytes land in <10ms but the divisor
        // was clamped to 50ms, not the 500ms over which those bytes should be
        // amortised.  Using BITRATE_WINDOW_MS as the floor gives a conservative
        // estimate that converges to the true rate after one full window.
        if (spanMs < BITRATE_WINDOW_MS) spanMs = BITRATE_WINDOW_MS;
        return (uint32_t)((m_totalBytes * 8ULL * 1000ULL) / (spanMs * 1000ULL));
    }

private:
    static const int MAX_BUCKETS = 600; // 1 per ms, 500ms window + margin
    struct Bucket { uint64_t timeMs; size_t bytes; };
    Bucket   m_buckets[MAX_BUCKETS];
    int      m_head;
    size_t   m_totalBytes;

    void expireOld(uint64_t nowMs)
    {
        uint64_t cutoff = (nowMs > BITRATE_WINDOW_MS) ? nowMs - BITRATE_WINDOW_MS : 0;
        int keep = 0;
        for (int i = 0; i < m_head; i++)
            if (m_buckets[i].timeMs >= cutoff) { m_buckets[keep++] = m_buckets[i]; }
            else m_totalBytes -= m_buckets[i].bytes;
        m_head = keep;
    }
};

static BitrateEstimator g_bitrateEstimator;

// =============================================================================
//  AdaptiveModCod — DVB-S2 FEC code rate adaptation for VBR transport streams
//
//  Primary signal: measured input bitrate (kbps) from BitrateEstimator.
//  Secondary signal: queue depth used only as a health check.
//  Underflow: immediate step-down + ceiling ratchet.
//  Ceiling: lowered on underflow, relaxed by +1 step every 30s of stability.
// =============================================================================

// ---------------------------------------------------------------------------
//  FEC table — sorted ascending by code rate (lowest rate = most robust).
// ---------------------------------------------------------------------------
struct FecEntry {
    uint8_t  fec;
    uint8_t  num;
    uint8_t  den;
    const char *label;
};

static const FecEntry kFecTable[] = {
    { C1_4,  1, 4,  "1/4"  },
    { C1_3,  1, 3,  "1/3"  },
    { C2_5,  2, 5,  "2/5"  },
    { C1_2,  1, 2,  "1/2"  },
    { C3_5,  3, 5,  "3/5"  },
    { C2_3,  2, 3,  "2/3"  },
    { C3_4,  3, 4,  "3/4"  },
    { C4_5,  4, 5,  "4/5"  },
    { C5_6,  5, 6,  "5/6"  },
    { C8_9,  8, 9,  "8/9"  },
    { C9_10, 9, 10, "9/10" },
};
static const int kFecCount = (int)(sizeof(kFecTable) / sizeof(kFecTable[0]));

// ---------------------------------------------------------------------------
//  Tuning parameters
// ---------------------------------------------------------------------------
#define AMODCOD_HEADROOM              0.10f   // 10% capacity margin above measured bitrate
#define AMODCOD_COOLDOWN_MS           400     // min ms between any two switches
#define AMODCOD_MIN_IDX               0
#define AMODCOD_MAX_IDX               (kFecCount - 1)
#define AMODCOD_UNDERFLOW_STEP        2       // step-down on underflow
#define AMODCOD_UNDERFLOW_PENALTY_MS  1000    // base upgrade freeze after underflow
#define AMODCOD_CEILING_RELAX_MS      30000   // ms of stability before ceiling rises +1
#define AMODCOD_QUEUE_URGENT          20      // queue depth above which upgrade jumps to ceiling
#define AMODCOD_QUEUE_HOLD             5      // queue depth below which downgrade is allowed

static int bitsPerSymbol(uint8_t constellation)
{
    switch (constellation)
    {
        case M_QPSK:   return 2;
        case M_8PSK:   return 3;
        case M_16APSK: return 4;
        case M_32APSK: return 5;
        default:       return 2;
    }
}

class AdaptiveModCod
{
public:
    AdaptiveModCod()
        : m_currentIdx(0), m_initialized(false)
        , m_underflowPenaltyActive(false)
        , m_ceilingIdx(AMODCOD_MAX_IDX)
        , m_upgradeConfirmCount(0)
        , m_pendingUpgradeIdx(-1)
        , m_symbolRateKsps(27000)
    {
        memset(&m_cooldownExpiry,         0, sizeof(m_cooldownExpiry));
        memset(&m_underflowPenaltyExpiry, 0, sizeof(m_underflowPenaltyExpiry));
        memset(&m_lastUnderflowTime,      0, sizeof(m_lastUnderflowTime));
        memset(&m_lastCeilingRelaxTime,   0, sizeof(m_lastCeilingRelaxTime));
    }

    // symbolRateKsps: actual TX symbol rate in ksps (= m_SRtx / 4000, FPGA has 4x interpolator)
    void init(const DVB2FrameFormat &baseFmt, uint32_t symbolRateKsps)
    {
        m_baseFmt                = baseFmt;
        m_symbolRateKsps         = symbolRateKsps;
        m_currentIdx             = fecToIdx(baseFmt.fec);
        m_underflowPenaltyActive = false;
        m_ceilingIdx             = AMODCOD_MAX_IDX;
        m_upgradeConfirmCount    = 0;
        m_pendingUpgradeIdx      = -1;
        m_initialized            = true;
        clock_gettime(CLOCK_MONOTONIC, &m_cooldownExpiry);
        clock_gettime(CLOCK_MONOTONIC, &m_lastUnderflowTime);
        clock_gettime(CLOCK_MONOTONIC, &m_lastCeilingRelaxTime);
        fprintf(stderr, "[AdaptiveModCod] init fec=%s (idx=%d) bps/sym=%d Rs=%uksps cap_max=%ukbps\n",
                kFecTable[m_currentIdx].label, m_currentIdx,
                bitsPerSymbol(baseFmt.constellation), m_symbolRateKsps,
                netCapacityKbps(AMODCOD_MAX_IDX));
    }

    void notifyUnderflow()
    {
        if (!m_initialized) return;
        int newCeiling = m_currentIdx - 1;
        if (newCeiling < AMODCOD_MIN_IDX) newCeiling = AMODCOD_MIN_IDX;
        if (newCeiling < m_ceilingIdx)    m_ceilingIdx = newCeiling;

        int newIdx = m_currentIdx - AMODCOD_UNDERFLOW_STEP;
        if (newIdx < AMODCOD_MIN_IDX) newIdx = AMODCOD_MIN_IDX;

        fprintf(stderr, "[AdaptiveModCod] UNDERFLOW: fec %s -> %s  (ceiling=%s)\n",
                kFecTable[m_currentIdx].label, kFecTable[newIdx].label,
                kFecTable[m_ceilingIdx].label);

        m_currentIdx          = newIdx;
        m_upgradeConfirmCount = 0;
        m_pendingUpgradeIdx   = -1;

        uint32_t R = g_bitrateEstimator.getRateKbps();
        uint32_t C = netCapacityKbps(m_currentIdx);
        long penaltyMs = AMODCOD_UNDERFLOW_PENALTY_MS;
        if (C > 0 && R > C)
            penaltyMs += (long)(((float)(R - C) / C) * 2000.0f);
        if (penaltyMs > 5000) penaltyMs = 5000;

        clock_gettime(CLOCK_MONOTONIC, &m_underflowPenaltyExpiry);
        m_underflowPenaltyExpiry.tv_sec  += penaltyMs / 1000;
        m_underflowPenaltyExpiry.tv_nsec += (penaltyMs % 1000) * 1000000L;
        if (m_underflowPenaltyExpiry.tv_nsec >= 1000000000L) {
            m_underflowPenaltyExpiry.tv_sec++;
            m_underflowPenaltyExpiry.tv_nsec -= 1000000000L;
        }
        m_underflowPenaltyActive = true;
        clock_gettime(CLOCK_MONOTONIC, &m_lastUnderflowTime);
        clock_gettime(CLOCK_MONOTONIC, &m_lastCeilingRelaxTime);
    }

    bool evaluate(size_t queueDepth, DVB2FrameFormat &activeFmt)
    {
        if (!m_initialized) { activeFmt = m_baseFmt; return false; }

        if (m_underflowPenaltyActive && elapsedMs(m_underflowPenaltyExpiry) >= 0)
            m_underflowPenaltyActive = false;

        if (m_ceilingIdx < AMODCOD_MAX_IDX &&
            elapsedMs(m_lastCeilingRelaxTime) >= AMODCOD_CEILING_RELAX_MS)
        {
            m_ceilingIdx++;
            clock_gettime(CLOCK_MONOTONIC, &m_lastCeilingRelaxTime);
            fprintf(stderr, "[AdaptiveModCod] ceiling relaxed to %s\n",
                    kFecTable[m_ceilingIdx].label);
        }

        uint32_t R_kbps = g_bitrateEstimator.getRateKbps();
        // Cap at maximum link capacity: bursts above cap_max can't be served at
        // any FEC rate, so asking for a higher-than-max target is meaningless and
        // causes the ceiling-trap to fire in a tight loop printing the same message.
        {
            uint32_t cap_max = netCapacityKbps(AMODCOD_MAX_IDX);
            if (R_kbps > cap_max) R_kbps = cap_max;
        }
        uint32_t R_needed = (uint32_t)(R_kbps * (1.0f + AMODCOD_HEADROOM));
        int targetIdx     = findMinSufficientIdx(R_needed);

        if (targetIdx > m_ceilingIdx) targetIdx = m_ceilingIdx;

        // Ceiling/floor trap: if ceiling == current and current is still
        // insufficient, jump the ceiling to the minimum sufficient rate.
        // Guard: only fire when the ceiling can actually move — when already at
        // AMODCOD_MAX_IDX the condition would otherwise repeat every BBframe.
        if (targetIdx == m_currentIdx
            && netCapacityKbps(m_currentIdx) < R_needed
            && m_ceilingIdx == m_currentIdx
            && m_ceilingIdx < AMODCOD_MAX_IDX
            && !m_underflowPenaltyActive)
        {
            int minSuff = findMinSufficientIdx(R_needed);
            m_ceilingIdx = (minSuff > m_ceilingIdx) ? minSuff : m_ceilingIdx + 1;
            if (m_ceilingIdx > AMODCOD_MAX_IDX) m_ceilingIdx = AMODCOD_MAX_IDX;
            targetIdx = m_ceilingIdx;
            fprintf(stderr, "[AdaptiveModCod] ceiling forced up to %s (insufficient FEC)\n",
                    kFecTable[m_ceilingIdx].label);
        }

        bool queueUrgent = ((int)queueDepth > AMODCOD_QUEUE_URGENT);
        bool queueLoaded = ((int)queueDepth > AMODCOD_QUEUE_HOLD);

        bool doSwitch = false;
        if (targetIdx > m_currentIdx)
        {
            if (queueUrgent)
            {
                // Queue is overloaded — jump directly to the ceiling in one step
                // instead of climbing one FEC index per BBframe.  This drains the
                // backlog as fast as possible without the staircase overhead.
                targetIdx = m_ceilingIdx;
                doSwitch = true;
            }
            else if (!m_underflowPenaltyActive)
            {
                if (m_pendingUpgradeIdx != targetIdx) {
                    m_pendingUpgradeIdx   = targetIdx;
                    m_upgradeConfirmCount = 0;
                }
                if (++m_upgradeConfirmCount >= 3 && cooldownExpired())
                    doSwitch = true;
            }
        }
       else if (targetIdx < m_currentIdx && R_kbps > 0 && cooldownExpired()
                 && !queueUrgent && !queueLoaded)
        {
            // Downgrade only when bitrate is real, cooldown elapsed, and the
            // queue has drained (≤ AMODCOD_QUEUE_HOLD).
            m_upgradeConfirmCount = 0;
            m_pendingUpgradeIdx   = -1;
            
            // --- PATCH: Slew-Rate Limiter ---
            // Instead of jumping directly to a low targetIdx, step down 
            // by exactly ONE index per evaluation window.
            targetIdx = m_currentIdx - 1; 
            
            doSwitch = true;
        }
        else
        {
            m_upgradeConfirmCount = 0;
        }

        if (doSwitch)
        {
            fprintf(stderr,
                "[AdaptiveModCod] fec %s -> %s  (R=%ukbps need=%ukbps cap=%ukbps q=%zu ceil=%s)\n",
                kFecTable[m_currentIdx].label, kFecTable[targetIdx].label,
                R_kbps, R_needed, netCapacityKbps(targetIdx), queueDepth,
                m_ceilingIdx < AMODCOD_MAX_IDX ? kFecTable[m_ceilingIdx].label : "none");
            m_currentIdx          = targetIdx;
            m_upgradeConfirmCount = 0;
            m_pendingUpgradeIdx   = -1;
            resetCooldown();
        }

        activeFmt     = m_baseFmt;
        activeFmt.fec = kFecTable[m_currentIdx].fec;
        return doSwitch;
    }

    const char *fecLabel() const { return m_initialized ? kFecTable[m_currentIdx].label : "uninit"; }

private:
    DVB2FrameFormat m_baseFmt;
    int             m_currentIdx;
    bool            m_initialized;
    bool            m_underflowPenaltyActive;
    int             m_ceilingIdx;
    int             m_upgradeConfirmCount;
    int             m_pendingUpgradeIdx;
    uint32_t        m_symbolRateKsps;
    struct timespec m_cooldownExpiry;
    struct timespec m_underflowPenaltyExpiry;
    struct timespec m_lastUnderflowTime;
    struct timespec m_lastCeilingRelaxTime;

    uint32_t netCapacityKbps(int idx) const
    {
        int bps = bitsPerSymbol(m_baseFmt.constellation);
        return (uint32_t)((uint64_t)m_symbolRateKsps * bps
                          * kFecTable[idx].num / kFecTable[idx].den);
    }

    int findMinSufficientIdx(uint32_t needed_kbps) const
    {
        for (int i = AMODCOD_MIN_IDX; i <= AMODCOD_MAX_IDX; i++)
            if (netCapacityKbps(i) >= needed_kbps) return i;
        return AMODCOD_MAX_IDX;
    }

    int fecToIdx(uint8_t fec) const
    {
        for (int i = 0; i < kFecCount; i++)
            if (kFecTable[i].fec == fec) return i;
        return AMODCOD_MIN_IDX;
    }

    long elapsedMs(const struct timespec &ts) const
    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        return (long)((now.tv_sec - ts.tv_sec) * 1000L
                    + (now.tv_nsec - ts.tv_nsec) / 1000000L);
    }

    bool cooldownExpired() const { return elapsedMs(m_cooldownExpiry) >= 0; }

    void resetCooldown()
    {
        clock_gettime(CLOCK_MONOTONIC, &m_cooldownExpiry);
        m_cooldownExpiry.tv_sec  += AMODCOD_COOLDOWN_MS / 1000;
        m_cooldownExpiry.tv_nsec += (AMODCOD_COOLDOWN_MS % 1000) * 1000000L;
        if (m_cooldownExpiry.tv_nsec >= 1000000000L) {
            m_cooldownExpiry.tv_sec++;
            m_cooldownExpiry.tv_nsec -= 1000000000L;
        }
    }
};
static AdaptiveModCod g_adaptiveModCod;
// ============================================================================= end AdaptiveModCod

// Plain wrapper callable from other translation units without exposing the class type
void adaptive_modcod_notify_underflow()
{
    g_adaptiveModCod.notifyUnderflow();
}

// ── Intermediate TS packet buffer ─────────────────────────────────────────────
// Decouples TS ingestion from BBframe encoding so the modcod decision is made
// at the START of each BBframe, before any of its TS packets enter the encoder.
// ─────────────────────────────────────────────────────────────────────────────
#define TS_PREBUF_MAX 512

struct TsEntry {
    uint8_t pkt[188];
    bool    isSdt;
};

static queue<TsEntry> g_ts_prebuf;
static bool            g_bbframe_at_start  = true;
static DVB2FrameFormat g_bbframe_active_fmt;
static bool            g_ts_discontinuity  = false; // set after a BBframe queue purge

static void prebuf_push(const uint8_t *src, bool isSdt)
{
    if ((int)g_ts_prebuf.size() >= TS_PREBUF_MAX)
    {
        fprintf(stderr, "TS prebuf overflow — dropping packet\n");
        return;
    }
    TsEntry e;
    memcpy(e.pkt, src, 188);
    e.isSdt = isSdt;
    g_ts_prebuf.push(e);
}

static void debug_ts_mirror_send(const uint8_t *pkt);

static void encode_from_prebuf()
{
    extern unsigned char getdvbs2modcod(uint FrameType, uint Constellation,
                                        uint CodeRate, uint Pilots);
    extern uint8_t  *customsdt;
    extern uint8_t   customnullpacket[188];
    extern unsigned int BBFrameLenLut2[22];
    extern void      update_cont_counter(uint8_t *b);
    extern bool      addbbframe(uint8_t *bbframe, size_t len, size_t modcod);
    extern size_t    m_SRtx;

    // Accumulated PCR correction (27 MHz ticks) for null packets removed from
    // the stream.  Each dropped 188-byte packet shortens the time before the
    // next real packet is on air by delta = 188*8*27e6 / net_bps ticks, so the
    // PCR of the next PCR-bearing packet must be decreased by that amount.
    static int64_t pcr_drop_correction = 0;

    while (!g_ts_prebuf.empty())
    {
        if (g_bbframe_at_start)
        {
            if (m_Fecmode == fec_variable)
                g_adaptiveModCod.evaluate(m_bbframe_queue.size(), g_bbframe_active_fmt);
            else
                g_bbframe_active_fmt = fmt;

            dvbs2neon_control(STREAM0, CONTROL_SET_PARAMETERS, (uint32)&g_bbframe_active_fmt, 0);
            m_efficiency = dvbs2neon_control(STREAM0, CONTROL_GET_EFFICIENCY, 0, 0);
            g_bbframe_at_start = false;
        }

        TsEntry e = g_ts_prebuf.front();
        g_ts_prebuf.pop();

        #ifdef WITH_PCR
        // Drop null packets here — they passed through ingestion for PCR
        // correction but must not consume BBframe capacity.
        if (GetPid((char *)e.pkt) == 0x1FFF)
        {
            // Compute net bitrate from the active BBframe format (always valid
            // here — the modcod block above runs before any packet is touched).
            int bps = bitsPerSymbol(g_bbframe_active_fmt.constellation);
            int fec_idx = 0;
            for (int k = 0; k < kFecCount; k++)
                if (kFecTable[k].fec == g_bbframe_active_fmt.fec) { fec_idx = k; break; }
            uint64_t net_bps = (uint64_t)(m_SRtx / 4) * bps
                               * kFecTable[fec_idx].num / kFecTable[fec_idx].den;
            if (net_bps > 0)
                pcr_drop_correction -= (int64_t)188 * 8 * 27000000LL / (int64_t)net_bps;
            continue;
        }

        // Apply accumulated PCR correction to the first subsequent packet that
        // carries a PCR, then reset so we don't double-apply.
        if (pcr_drop_correction != 0 && PCRAvailable((char *)e.pkt))
        {
            int64_t corrected = (int64_t)GetPCRFromPacket(e.pkt) + pcr_drop_correction;
            if (corrected < 0) corrected = 0;
            SetPacketPCR(e.pkt, (uint64_t)corrected);
            pcr_drop_correction = 0;
        }

        // Stamp the continuity counter for stuffing packets at encoding time
        // so the counter reflects what actually goes on air.  Also accumulate a
        // positive PCR correction: inserting a stuffing packet pushes subsequent
        // real packets one slot later, so the next PCR must be advanced by delta.
        if (GetPid((char *)e.pkt) == 0x1FFE)
        {
            static uint8_t cc_stuffing = 0;
            e.pkt[3] = (e.pkt[3] & 0xF0) | cc_stuffing;
            cc_stuffing = (cc_stuffing + 1) & 0x0F;

            int bps = bitsPerSymbol(g_bbframe_active_fmt.constellation);
            int fec_idx = 0;
            for (int k = 0; k < kFecCount; k++)
                if (kFecTable[k].fec == g_bbframe_active_fmt.fec) { fec_idx = k; break; }
            uint64_t net_bps = (uint64_t)(m_SRtx / 4) * bps
                               * kFecTable[fec_idx].num / kFecTable[fec_idx].den;
            if (net_bps > 0)
                pcr_drop_correction += (int64_t)188 * 8 * 27000000LL / (int64_t)net_bps;
        }
        
        if (e.isSdt)
            update_cont_counter(customsdt);

        if (GetPid((char *)e.pkt) != 0x1FFF && GetPid((char *)e.pkt) != 0x1FFE)
        {
            // correctcc rewrites CC to maintain continuity; after a queue purge
            // it also sets the discontinuity_indicator so the receiver knows
            // data was intentionally skipped rather than counting it as an error.
            correctcc(e.pkt, g_ts_discontinuity);
            g_ts_discontinuity = false;
        }
        #endif
        debug_ts_mirror_send(e.pkt);
        unsigned short *bbframeptr =
            (unsigned short *)dvbs2neon_packet(0, (uint32)(e.pkt), 0);

        if (bbframeptr != NULL)
        {
            unsigned short *p16      = (unsigned short *)bbframeptr;
            unsigned short ByteCount = p16[-1];

            // Compute expected BBframe byte count from the active format so we
            // can reject a stale or corrupted NEON output before it overflows
            // the 8000-byte buffer in buffer_t.
            int exp_fec_idx = 0;
            for (int k = 0; k < kFecCount; k++)
                if (kFecTable[k].fec == g_bbframe_active_fmt.fec) { exp_fec_idx = k; break; }
            int exp_lut_idx = (g_bbframe_active_fmt.frame_type == FRAME_NORMAL)
                              ? exp_fec_idx + 11 : exp_fec_idx;
            uint16_t expected_bytes = (exp_lut_idx >= 0 &&
                                       exp_lut_idx < (int)(sizeof(BBFrameLenLut2)/sizeof(BBFrameLenLut2[0])))
                                      ? BBFrameLenLut2[exp_lut_idx] / 8 : 0;

            if (ByteCount == 0 || ByteCount > UDP_BUFF_MAX_BBFRAME || ByteCount != expected_bytes)
            {
                fprintf(stderr, "encode_from_prebuf: bad BBframe size %u (expected %u) — discarding\n",
                        ByteCount, expected_bytes);
                g_bbframe_at_start = true;
                continue;
            }

            int i = 0;
            for (; i < (int)(sizeof(BBFrameLenLut2) / sizeof(BBFrameLenLut2[0])); i++)
                if (BBFrameLenLut2[i] / 8 == ByteCount) break;
            int coderate = (i >= 11) ? i - 11 : i;

            unsigned char curmodcod = getdvbs2modcod(
                g_bbframe_active_fmt.frame_type == 0 ? 0 : 1,
                g_bbframe_active_fmt.constellation,
                g_bbframe_active_fmt.fec,
                g_bbframe_active_fmt.pilots);

            m_variable_ts_coderate = (coderate < kFecCount) ? coderate : 0;
            addbbframe((uint8_t *)bbframeptr, ByteCount, curmodcod);

            g_bbframe_at_start = true;
        }
    }
}

void adaptive_modcod_update_sr(uint32_t srKsps)
{
    g_adaptiveModCod.init(fmt, srKsps);
    g_bbframe_at_start = true;
}

void flush_ts_prebuf()
{
    while (!g_ts_prebuf.empty())
        g_ts_prebuf.pop();
    g_bbframe_at_start = true;
}

enum
{
    tssource_udp,
    tssource_file,
    tssource_pattern
};

enum
{
    tx_passtrough,
    tx_iq,
    tx_dvbs2_ts,
    tx_dvbs2_gse,
    tx_test,
    tx_dvbs,
    tx_dvbt

};

int m_tssource = tssource_udp;
char m_mcast_ts[255];
char m_mcast_iface[255];
char m_ts_filename[255];
uint m_ts_filebitrate = 10000;
FILE *fdtsinput = NULL;
int m_FileBitrate = 100000;
int m_LatencySevenPacket = 1000;
size_t Lastpidccerror = 8192;

bool addbbframe(uint8_t *bbframe, size_t len, size_t modcod)
{
    if ((m_txmode != tx_dvbs2_ts)&&(m_txmode != tx_dvbs))
        return false;
    pthread_mutex_lock(&buffer_mutextx);
    buffer_t *newbuf = (buffer_t *)malloc(sizeof(buffer_t));
    newbuf->size = len;
    newbuf->modecod = modcod;
    // newbuf->modecod=m_ModeCod;
    memcpy(newbuf->bbframe, bbframe, len);

    /*
    if ((m_bbframe_queue.size() > MAX_QUEUE_CHANGEMODCOD)&&(m_bbframe_queue.size() < MAX_QUEUE_ITEM))
    {
        fprintf(stderr, "Queue is full, change modecod ! \n");
        DVB2FrameFormat tempmodecode;
        tempmodecode=fmt;
        tempmodecode.fec+=1;
        newbuf->modecod=m_ModeCod+1;
        int status = dvbs2neon_control(STREAM0, CONTROL_SET_PARAMETERS, (uint32)&tempmodecode, 0);
    }
    */
    if ((m_bbframe_queue.size() >= MAX_QUEUE_ITEM))
    {
        fprintf(stderr, "MUXTS : Queue is full ! Purging %zu bbframe\n", m_bbframe_queue.size());
        while (m_bbframe_queue.size() > 1)
        {
            buffer_t *oldestbuf = m_bbframe_queue.front();
            free(oldestbuf);
            m_bbframe_queue.pop();
        }
        // Queue overflow is as severe as a DMA underflow: the encoder cannot
        // drain BBframes fast enough for the current FEC rate. Penalise the
        // ceiling and force a step-down so the controller reacts immediately.
        if (m_Fecmode == fec_variable)
            g_adaptiveModCod.notifyUnderflow();
        g_ts_discontinuity = true;
    }

    /*if (m_bbframe_queue.size() <= MAX_QUEUE_CHANGEMODCOD)
    {

        int status = dvbs2neon_control(STREAM0, CONTROL_SET_PARAMETERS, (uint32)&fmt, 0);
    }
    */
   //fprintf(stderr,"Add bbframe of %d\n",newbuf->size);
    m_bbframe_queue.push(newbuf);
    pthread_mutex_unlock(&buffer_mutextx);
    return true;
}

size_t udp_receive(u_int16_t sock, unsigned char *b, unsigned int maxlen)
{

    size_t rcvlen = recv(sock, b, maxlen, 0 /* MSG_ZEROCOPY*/);
    return rcvlen;
}

u_int16_t udp_init(char *ip, const char *iface, int rx) // interface to multicast from
{

    u_int16_t sock;
    struct sockaddr_in addr;

    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

    /* try to reuse the socket */
    int len = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &len, sizeof(len));

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 10000; //10ms timeout
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);

    char text[40];
    char *add[2];
    u_int16_t port;
    strcpy(text, ip);
    add[0] = strtok(text, ":");
    add[1] = strtok(NULL, ":");
    if (strlen(add[1]) == 0)
        port = 1314;
    else
        port = atoi(add[1]);
    /* bind the socket on given port */
    memset(&addr, 0, sizeof(addr));           // Clear struct
    addr.sin_family = AF_INET;                // Internet/IP
    addr.sin_addr.s_addr = inet_addr(add[0]); // IP address
    addr.sin_port = htons(port);

    bind(sock, (struct sockaddr *)&addr, sizeof(addr));

    if (rx)
    {
        struct ip_mreq imr;
        memset(&imr, 0, sizeof(struct ip_mreq));
        inet_aton(add[0], (struct in_addr *)&(imr.imr_multiaddr.s_addr));
        // imr.imr_multiaddr = inet_addr(add[0]);
        if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (void *)&imr,
                       sizeof(struct ip_mreq)) < 0)
        {
            perror("setsockopt: join multicast group with IP_ADD_MEMBERSHIP failed\n");
            // exit(1);
        }
    }
    if (iface != NULL)
    {
        struct in_addr localInterface;
        localInterface.s_addr = inet_addr(iface);
        setsockopt(sock, IPPROTO_IP, IP_MULTICAST_IF, (char *)&localInterface, sizeof(localInterface));
    }
    else
    {
        setsockopt(sock, IPPROTO_IP, IP_MULTICAST_IF, NULL, 0);
    }
    int rcvbuf = 2*1024 * 1024; // 1 Megabyte buffer
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    return sock;
}

// ── Debug TS mirror ───────────────────────────────────────────────────────────
// Mirrors every TS packet entering the BBframe encoder to UDP multicast
// 230.0.0.2:1234, batched into standard 7-packet (1316-byte) datagrams.
// ─────────────────────────────────────────────────────────────────────────────
#define DEBUG_TS_MCAST_IP   "230.0.0.2"
#define DEBUG_TS_MCAST_PORT 1234
#define DEBUG_TS_UDP_PKTS   7

static int               g_debug_ts_sock  = -1;
static struct sockaddr_in g_debug_ts_addr;
static uint8_t           g_debug_ts_buf[188 * DEBUG_TS_UDP_PKTS];
static int               g_debug_ts_count = 0;

static void debug_ts_mirror_init(void)
{
    g_debug_ts_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (g_debug_ts_sock < 0) return;
    unsigned char ttl = 1;
    setsockopt(g_debug_ts_sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));
    memset(&g_debug_ts_addr, 0, sizeof(g_debug_ts_addr));
    g_debug_ts_addr.sin_family      = AF_INET;
    g_debug_ts_addr.sin_addr.s_addr = inet_addr(DEBUG_TS_MCAST_IP);
    g_debug_ts_addr.sin_port        = htons(DEBUG_TS_MCAST_PORT);
}

static void debug_ts_mirror_send(const uint8_t *pkt)
{
    if (g_debug_ts_sock < 0) debug_ts_mirror_init();
    if (g_debug_ts_sock < 0) return;
    memcpy(g_debug_ts_buf + g_debug_ts_count * 188, pkt, 188);
    if (++g_debug_ts_count >= DEBUG_TS_UDP_PKTS)
    {
        sendto(g_debug_ts_sock, g_debug_ts_buf, sizeof(g_debug_ts_buf), 0,
               (struct sockaddr *)&g_debug_ts_addr, sizeof(g_debug_ts_addr));
        g_debug_ts_count = 0;
    }
}

#define CRC_POLYR 0xD5
uint8_t m_crc_tab_r[256];
void build_crc8_table_r(void)
{
    int r, crc;

    for (int i = 0; i < 256; i++)
    {
        r = i;
        crc = 0;
        for (int j = 7; j >= 0; j--)
        {
            if ((r & (1 << j) ? 1 : 0) ^ ((crc & 0x80) ? 1 : 0))
                crc = (crc << 1) ^ CRC_POLYR;
            else
                crc <<= 1;
        }
        m_crc_tab_r[i] = crc;
    }
}

uint8_t calc_crc8_r(uint8_t *b, int len)
{
    uint8_t crc = 0;

    for (int i = 0; i < len; i++)
    {
        crc = m_crc_tab_r[b[i] ^ crc];
    }
    return crc;
}

#define MAX_BBFRAME (58192 / 8)

unsigned int BBFrameLenLut2[] = {3072, 5232, 6312, 7032, 9552, 10632, 11712, 12432, 13152, 14232, 0, 16008, 21408, 25728, 32208, 38688, 43040, 48408, 51648, 53840, 57472, 58192};
// extern unsigned int BBFrameLenLut[];

u_int16_t recv_ts_sock;

pthread_mutex_t buffer_mutexts;
DVB2FrameFormat tempmodecode;
uint8_t *customsdt;
uint8_t customnullpacket[188];

uint8_t *sdt_fmt(int stream_id, int network_id, int service_id, char *service_provider_name, char *service_name);
void update_cont_counter(uint8_t *b);

void correctcc(uint8_t *tspacket, bool discountinuity)
{
    static uint8_t pid_cc_table[8190];
    static int start = 1;
    unsigned short pid;
    if (start == 1)
    {
        memset(pid_cc_table, 0x10, 8190);
        start = 0;
    }

    memcpy(&pid, tspacket + 1, 2);
    pid = ntohs(pid);
    pid = pid & 0x1fff;
    // fprintf(stderr, "pid entry %d\n", pid);
    if (pid < 8190)
    {
        if (pid_cc_table[pid] == 0x10)
        {
            // fprintf(stderr, "new pid entry %d\n", pid);
            pid_cc_table[pid] = 0;
        }
        else
        {
            uint8_t afc = tspacket[3] & 0x30;
            if (afc == 0x10 || afc == 0x30)
            {
                pid_cc_table[pid] = (pid_cc_table[pid] + 1) % 0x10;
                // adaptation+payload: adaptation field present, set discontinuity_indicator
                if (discountinuity && afc == 0x30 && tspacket[4] > 0)
                    tspacket[5] |= 0x80;
            }
            else // adaptation-only (0x20) or reserved (0x00)
            {
                if (discountinuity && tspacket[4] > 0)
                    tspacket[5] |= 0x80;
            }
            // fprintf(stderr, "pid %d cc %d\n", pid,pid_cc_table[pid]);
        }

        tspacket[3] = (pid_cc_table[pid] | (tspacket[3] & 0xf0));
    }
}

void addneonts(uint8_t *tspacket, size_t length)
{
    if (length % 188 != 0)
    {
        fprintf(stderr, "Ts input error len %d\n", length);
        return;
    }

    // ── Phase 1: Ingest ───────────────────────────────────────────────────────
    // Preprocess each TS packet and push it into the intermediate buffer.
    // No encoding decisions here — the encoder is not touched in this phase.
    uint8_t *cur_packet = tspacket;
    for (int i = 0; i < (int)(length / 188); i++, cur_packet += 188)
    {
        if (cur_packet[0] != 0x47)
        {
            fprintf(stderr, "Ts input error aligned %x\n", cur_packet[0]);
            break;   // skip the rest of this batch but still encode what was already buffered
        }

        // Count only real content: exclude null packets (0x1FFF) and internal
        // stuffing (0x1FFE from setpaddingts). Both are still buffered and PCR-
        // corrected; nulls are dropped at the encoder boundary in encode_from_prebuf.
        if (GetPid((char *)cur_packet) != 0x1FFF && GetPid((char *)cur_packet) != 0x1FFE)
            g_bitrateEstimator.addBytes(188);
        //ProcessCorectPCR(cur_packet, 188); //Will be later

        if (GetPid((char *)cur_packet) == 0x11)
        {
            // SDT: buffer the replacement packet; counter update deferred to encode phase
            prebuf_push(customsdt, true);
        }
        /*
        else if (GetPid((char *)cur_packet) == 0x1FFF)
        {
            prebuf_push(customnullpacket, false);
        }*/
        else
        {
            /*size_t piderror = InspectCC(cur_packet, 188);
            if (piderror != 8192)
                Lastpidccerror = piderror;*/
            prebuf_push(cur_packet, false);
        }
    }

    // ── Phase 2: Encode ───────────────────────────────────────────────────────
    // Drain the intermediate buffer into BBframes. The modcod is chosen once
    // at the start of each BBframe, before any of its TS packets enter the
    // encoder, so SET_PARAMETERS is always applied at a clean boundary.
    encode_from_prebuf();
}

void adddvbsframe(uint8_t *tspacket)
{
    static unsigned char Buff[208];
    static unsigned char *pRS204;
    static unsigned char packedSymbol[408]; // 204*2bits/symbol /8 (byte)
    static unsigned char symbols[408 * 4];  // 204*2bits/symbol  (byte)
    static uint16_t Frame[408*4*2];
    uint16 NbIQOutput;
    uint16 NbSymbolsTotal = 0;
    uint16_t *FrameCurrentSymbol=Frame;
    uint16_t magnitude=0x7FFF/sqrt(2);
    memcpy(Buff, tspacket, 188);

    pRS204 = dvbsenco(Buff);
    NbIQOutput = viterbi(pRS204, packedSymbol);
    //fprintf(stderr,"viterbi out %d\n",NbIQOutput);
    for (int k = 0; k < NbIQOutput; k++)
    {
        for (int j = 0; j < 4; j++)
        {
            //symbols[k * 4 + j] = (packedSymbol[k] >> (6 - j * 2)) & 0x3;
            switch((packedSymbol[k] >> (6 - j * 2)) & 0x3)
            {
                case 0: *FrameCurrentSymbol++=magnitude;*FrameCurrentSymbol++=magnitude;break;
                case 1: *FrameCurrentSymbol++=magnitude;*FrameCurrentSymbol++=-magnitude;break;
                case 2: *FrameCurrentSymbol++=-magnitude;*FrameCurrentSymbol++=magnitude;break;
                case 3: *FrameCurrentSymbol++=-magnitude;*FrameCurrentSymbol++=-magnitude;break;
            }
            NbSymbolsTotal++;
            //fprintf(stderr,"Symbol %d -> %d",(packedSymbol[k] >> (6 - j * 2)) & 0x3,*(FrameCurrentSymbol-1));
        }
    }
    //fprintf(stderr,"Nb IQ %d\n",NbSymbolsTotal);
    addbbframe((uint8_t *)Frame,NbSymbolsTotal*4,0); //IQ = 4 bytes
}

void adddvbsts(uint8_t *tspacket, size_t length)
{

       uint8_t *cur_packet = tspacket;

    if (length % 188 != 0)
    {
        fprintf(stderr, "Ts input error len %d\n", length);
        return;
    }
    for (int i = 0; i < length / 188; i++)
    {
        // pthread_mutex_lock(&buffer_mutexts);
        if (cur_packet[0] != 0x47)
        {
            fprintf(stderr, "Ts input error aligned %x\n", cur_packet[0]);
            return;
        }
       
        ProcessCorectPCR(cur_packet, 188); // Correct PCR

        if (GetPid((char *)cur_packet) == 0x11) // replace sdt
        {
            // bbframeptr = (unsigned short *) dvbs2neon_packet(0, (uint32)(customsdt), 0);
            adddvbsframe(customsdt);
            update_cont_counter(customsdt);
            debug_ts_mirror_send(customsdt);
        }
        else if (GetPid((char *)cur_packet) == 0x1FFF)
        {
            adddvbsframe(customnullpacket);
            // bbframeptr = (unsigned short *) dvbs2neon_packet(0, (uint32)(customnullpacket), 0);
            debug_ts_mirror_send(customnullpacket);
        }
        else
        {
            size_t piderror = InspectCC(cur_packet, 188);
            if (piderror != 8192)
                Lastpidccerror = piderror;
            adddvbsframe(cur_packet);
            // bbframeptr = (unsigned short *) dvbs2neon_packet(0, (uint32)(cur_packet), 0);
            debug_ts_mirror_send(cur_packet);
        }
        cur_packet += 188;
       
    }
}

void addts(uint8_t *tspacket, size_t length)
{
    static unsigned char bbframe[MAX_BBFRAME];
    // BBFrameLenLut2 layout: indices 0-10 = short-frame DFLs (C1/4..C9/10),
    // indices 11-21 = long-frame DFLs.  longframe=0 must add 11 to reach the
    // long-frame half of the table; shortframe=1 uses the short-frame half directly.
    int bbfLutIdx = (fmt.frame_type == FRAME_NORMAL) ? (m_ModeCod + 11) : m_ModeCod;
    if (bbfLutIdx < 0 || bbfLutIdx >= (int)(sizeof(BBFrameLenLut2)/sizeof(BBFrameLenLut2[0])))
        bbfLutIdx = 16; // safe fallback: long C2/3
    uint16_t framebytes = BBFrameLenLut2[bbfLutIdx] / 8;
    static uint16_t avail = framebytes - 10;
    static uint8_t lastcrc = 0;
    struct bbheader *header = (struct bbheader *)bbframe;
    static int index = 10;
    static uint16_t remain = 0;

    uint16_t tsindex = 0;

    while (tsindex < length)
    {
        uint8_t crc = calc_crc8_r(tspacket + tsindex + 1, 187);
        if (avail >= 188)
        {

            /*
            bbframe[index++] = lastcrc;
            lastcrc = crc;
            avail--;*/
            memcpy(bbframe + index, tspacket + tsindex + 1, 188 - 1);
            tsindex += 188;
            index += 187;
            avail -= 187;
            bbframe[index++] = crc;
            avail--;
        }
        else if (avail == 0) // BBFrame is full
        {
            // sendudpbbframe
            // fprintf(stderr,"Complete full\n");
            if (remain == 0)
                remain = 187;
            header->matype1 = 0xF0; // TS 0.35roff

            header->matype2 = 0;                        // Input Stream Identifier
            header->upl = htons(188 * 8);               // User Packet Length 188
            header->dfl = htons((framebytes - 10) * 8); // Data Field Length
            header->sync = 0x47;                        // SYNC - Copy of the user packet Sync byte
            header->syncd1 = (remain * 8) >> 8;         // SYNCD
            header->syncd2 = (remain * 8) & 0xFF;       // SYNCD
            header->crc = calc_crc8_r(bbframe, 9);      // CRC

            // udp_send(send_bbframe_sock, send_bbframe_ip, bbframe, framebytes);
            addbbframe(bbframe, framebytes, m_ModeCod);

            index = 10;

            avail = framebytes - 10;
        }
        else // Some space in bbframe, full with partial packet
        {

            /*
           bbframe[index++] = lastcrc;
           lastcrc = crc;
          */
            if (remain == 0)
                remain = 187;
            /*
            else
            remain--;
            */
            header->matype1 = 0xF0; // TS 0.35roff

            header->matype2 = 0;                        // Input Stream Identifier
            header->upl = htons(188 * 8);               // User Packet Length 188
            header->dfl = htons((framebytes - 10) * 8); // Data Field Length
            header->sync = 0x47;                        // SYNC - Copy of the user packet Sync byte
            header->syncd1 = (remain * 8) >> 8;         // SYNCD
            header->syncd2 = (remain * 8) & 0xFF;       // SYNCD
            header->crc = calc_crc8_r(bbframe, 9);      // CRC

            remain = 187 - avail;
            memcpy(bbframe + index, tspacket + tsindex + 1, avail);
            index += avail;
            // fprintf(stderr,"Remain full remain %d %d\n",avail,index);

            /*
            static int count=0;
                for(int i=0;i<  framebytes;i++)
          fprintf(stderr,"%02x ", bbframe[i]);
          fprintf(stderr,"\n************************************************\n");

          if(count++==2)          exit(1);
          */

            // udp_send(send_bbframe_sock, send_bbframe_ip, bbframe, framebytes);
            addbbframe(bbframe, framebytes, m_ModeCod);
            // sendudpbbframe

            index = 10;
            memcpy(bbframe + index, tspacket + tsindex + avail + 1, remain);
            index += remain;
            bbframe[index++] = crc;
            tsindex += 188;

            avail = framebytes - 10 - remain - 1;
        }
    }
}

#define MAX_PID 8191
#define TS_PACKET_SIZE 188
#define SYSTEM_CLOCK_FREQUENCY 27000000

int GetTsBitrate()
{
    // Courtsy taken from OpenCaster toolbox , GPL
    int fd_ts; /* File descriptor of ts file */
    u_short pid;
    int byte_read;
    unsigned int pcr_ext = 0;
    unsigned int ibits = 0;
    unsigned long long int pcr_base = 0;
    unsigned long long int ts_packet_count;
    unsigned long long int new_pcr = 0;
    unsigned long long int new_pcr_index = 0;
    unsigned long long int pid_pcr_table[MAX_PID];       /* PCR table for the TS packets */
    unsigned long long int pid_pcr_index_table[MAX_PID]; /* PCR index table for the TS packets */
    unsigned long long int pid_sum[MAX_PID];             /* Sum of each PID TS packets */
    unsigned char ts_packet[TS_PACKET_SIZE];             /* TS packet */

    unsigned long long int TsBitrate = 0;

    /* Start to process the file */
    memset(pid_pcr_table, 0, MAX_PID * (sizeof(unsigned long long int)));
    memset(pid_pcr_index_table, 0, MAX_PID * (sizeof(unsigned long long int)));
    memset(pid_sum, 0, MAX_PID * (sizeof(unsigned long long int)));
    ts_packet_count = 0;
    byte_read = 1;
    int nbnewpcr = 0;
    while (TsBitrate == 0)
    {

        /* Read next packet */

        byte_read = fread(ts_packet, TS_PACKET_SIZE, 1, fdtsinput);
        /* check packet */
        memcpy(&pid, ts_packet + 1, 2);
        pid = ntohs(pid);
        pid = pid & 0x1fff;
        if (pid < MAX_PID)
        {
            if ((ts_packet[3] & 0x20) && (ts_packet[4] != 0) && (ts_packet[5] & 0x10))
            { /* there is a pcr field */
                pcr_base = (((unsigned long long int)ts_packet[6]) << 25) + (ts_packet[7] << 17) + (ts_packet[8] << 9) + (ts_packet[9] << 1) + (ts_packet[10] >> 7);
                pcr_ext = ((ts_packet[10] & 1) << 8) + ts_packet[11];
                if (pid_pcr_table[pid] == 0)
                {
                    pid_pcr_table[pid] = pcr_base * 300 + pcr_ext;
                    pid_pcr_index_table[pid] = (ts_packet_count * TS_PACKET_SIZE) + 10;
                    /*fprintf(stdout, "%llu: pid %d, new pcr is %llu (%f sec)\n",
                                    pid_pcr_index_table[pid],
                                    pid,
                                    pid_pcr_table[pid],
                                    ((double)(pid_pcr_table[pid]) / SYSTEM_CLOCK_FREQUENCY));*/
                }
                else
                {

                    new_pcr = pcr_base * 300 + pcr_ext;
                    new_pcr_index = (ts_packet_count * TS_PACKET_SIZE) + 10;
                    /*fprintf(stderr, "%llu: pid %d, new pcr is %llu (%f sec), pcr delta is %llu, (%f ms), indices delta is %llu bytes,instant ts bit rate is %.10f\n",
                                    new_pcr_index,
                                    pid,
                                    new_pcr,
                                    ((double)(new_pcr) / SYSTEM_CLOCK_FREQUENCY),
                                    new_pcr - pid_pcr_table[pid],
                                    ((double)((new_pcr - pid_pcr_table[pid]) * 1000)) / SYSTEM_CLOCK_FREQUENCY,
                                    new_pcr_index - pid_pcr_index_table[pid],
                                    (((double)(new_pcr_index - pid_pcr_index_table[pid])) * 8 * SYSTEM_CLOCK_FREQUENCY) / ((double)(new_pcr - pid_pcr_table[pid]))
                                    );*/

                    double ftsbitrate = (((double)(new_pcr_index - pid_pcr_index_table[pid])) * 8 * SYSTEM_CLOCK_FREQUENCY) / ((double)(new_pcr - pid_pcr_table[pid]));
                    nbnewpcr++;
                    if (nbnewpcr > 8) // Wait 10 pcr for average
                        TsBitrate = (unsigned long long int)ftsbitrate;
                    unsigned long long int videobitrate = (TsBitrate * pid_sum[pid]) / ts_packet_count;
                    // fprintf(stderr, "Ts %f %ll Video %ll\n", ftsbitrate, TsBitrate, videobitrate);

                    pid_pcr_table[pid] = new_pcr;
                    pid_pcr_index_table[pid] = new_pcr_index;
                }
            }
            pid_sum[pid]++;
            ts_packet_count++;
        }
    }
    fseek(fdtsinput, 0, SEEK_SET);
    return TsBitrate;
}

// Returns true when the UDP receive socket has at least one TS packet pending.
// Called from the TX thread before deciding to insert a stuffing packet.
bool ts_udp_data_pending()
{
    if (m_tssource != tssource_udp) return false;
    int avail = 0;
    ioctl(recv_ts_sock, FIONREAD, &avail);
    return avail >= 188;
}

void *rx_ts_thread(void *arg)
{
    unsigned char tspacket[7 * 188];

    int length = 0;

    while (true)
    {
        if (m_tssource == tssource_udp)
            length = udp_receive(recv_ts_sock, tspacket, sizeof(tspacket));

        if ((m_tssource == tssource_file) || (m_tssource == tssource_pattern))
        {
            if (fdtsinput == NULL)
            {
                usleep(5000);
                continue;
            }
            if (feof(fdtsinput)) // Loop File
            {
                fseek(fdtsinput, 0, SEEK_SET);
                // correctcc(tspacket,true);
            }
            if (m_bbframe_queue.size() < 10)
            {
                length = fread(tspacket, 1, sizeof(tspacket), fdtsinput);
            }
            else
            {
                usleep(m_LatencySevenPacket * 1000);
                continue;
            }
        }

        if (length > 0)
        {
            pthread_mutex_lock(&buffer_mutexts);
            if (m_txmode == tx_dvbs2_ts)
                addneonts(tspacket, length);
            if (m_txmode == tx_dvbs)
                adddvbsts(tspacket, length);
            pthread_mutex_unlock(&buffer_mutexts);

            // Drain any additional UDP datagrams that queued up while we held
            // the mutex or were processing.  Each extra datagram is processed
            // individually to avoid any buffer-size constraint.
            if (m_tssource == tssource_udp)
            {
                unsigned char extra[7 * 188];
                ssize_t extra_len;
                while ((extra_len = recv(recv_ts_sock, extra, sizeof(extra), MSG_DONTWAIT)) > 0)
                {
                    pthread_mutex_lock(&buffer_mutexts);
                    if (m_txmode == tx_dvbs2_ts)
                        addneonts(extra, (int)extra_len);
                    if (m_txmode == tx_dvbs)
                        adddvbsts(extra, (int)extra_len);
                    pthread_mutex_unlock(&buffer_mutexts);
                }
            }
        }
    }
}

void setneonmodcod(uint Constellation, uint CodeRate, uint FrameType, uint Pilots, int filter)
{

    switch (Constellation)
    {
    case mod_qpsk:
        fmt.constellation = M_QPSK;
        break;
    case mod_8psk:
        fmt.constellation = M_8PSK;
        break;
    case mod_16apsk:
        fmt.constellation = M_16APSK;
        break;
    case mod_32apsk:
        fmt.constellation = M_32APSK;
        break;
    default:
        fmt.constellation = M_QPSK;
    }
    fmt.fec = CodeRate;
    fmt.frame_type = (FrameType == shortframe) ? FRAME_SHORT : FRAME_NORMAL;
    fmt.output_format = OUTPUT_FORMAT_BBFRAME;
    if (Pilots)
        fmt.pilots = PILOTS_ON;
    else
        fmt.pilots = PILOTS_OFF;
    if (filter == 0)
        fmt.roll_off = RO_0_20;
    if (filter == 1)
        fmt.roll_off = RO_0_15;

    int status = dvbs2neon_control(STREAM0, CONTROL_SET_PARAMETERS, (uint32)&fmt, 0);
    modulator_mapping(fmt.constellation, CodeRate);

    m_efficiency = dvbs2neon_control(STREAM0, CONTROL_GET_EFFICIENCY, 0, 0);

    // Re-seed the adaptive controller so it uses the new operator baseline
    { extern size_t m_SRtx; g_adaptiveModCod.init(fmt, (uint32_t)(m_SRtx / 4000)); }
    g_bbframe_at_start = true;
    // pthread_mutex_unlock(&buffer_mutexts);
}

void setdvbsfec(uint CodeRate)
{
    switch(CodeRate) //Retrict fec allowed in dvbs
    {
        case C1_4:
        case C1_3:
        case C2_5:
        case C1_2: CodeRate=1; m_efficiency=(1e6*188*2)/(2*204);break;

    case C3_5:
    case C2_3: CodeRate=2;m_efficiency=(1e6*188*2*2)/(3*204);break;
    case C3_4 : CodeRate=3;m_efficiency=(1e6*188*3*2)/(4*204);break;
    case C4_5 : 
    case C5_6 :CodeRate=5;m_efficiency=(1e6*188*5*2)/(6*204);break;
    case C8_9 :
    case C9_10 : CodeRate=7;m_efficiency=(1e6*188*7*2)/(8*204);break;
    }
    fprintf(stderr,"DVBS FEC %d\n",CodeRate);
    dvbsenco_init();
    viterbi_init(CodeRate); // 1/2
    
}

void settssource(int tssource, char *arg, uint tsbitrate)
{
    if (tssource != -1)
    {
        m_tssource = tssource;
        fprintf(stderr, "Change source %d\n", m_tssource);
    }
    switch (m_tssource)
    {
    case tssource_udp:
    {
        if (arg == NULL)
            break;
        strcpy(m_mcast_ts, arg);
        fprintf(stderr, "Try udp %s\n", m_mcast_ts);
        // fixmme should close previous socket
        recv_ts_sock = udp_init(m_mcast_ts, m_mcast_iface, 1);
    }
    break;
    case tssource_file:
    {
        if (arg != NULL)
        {
            fprintf(stderr, "Setting file ts %s\n", arg);
            strcpy(m_ts_filename, arg);
            if (fdtsinput != NULL)
                fclose(fdtsinput);
            fdtsinput = fopen(m_ts_filename, "rb");
            m_FileBitrate = GetTsBitrate();
            m_LatencySevenPacket = (1000 * 7 * 188 * 8L) / (long)m_FileBitrate;
            fprintf(stderr, "File bitrate = %d Latency us %d\n", m_FileBitrate, m_LatencySevenPacket * 1000);
        }
        if (tsbitrate > 0)
        {
            fprintf(stderr, "Setting file ts bitrate %d\n", tsbitrate);
            m_FileBitrate = tsbitrate;
            m_LatencySevenPacket = (1000 * 7 * 188 * 8L) / (long)m_FileBitrate;
        }
    }
    break;
    case tssource_pattern:
    {
        if (fdtsinput != NULL)
            fclose(fdtsinput);
        fdtsinput = fopen("/root/mire.ts", "rb");
        m_FileBitrate = GetTsBitrate();
        if (m_FileBitrate > 0)
            m_LatencySevenPacket = (1000 * 7 * 188 * 8L) / (long)m_FileBitrate;

        fprintf(stderr, "Patern bitrate = %d Latency us %d\n", m_FileBitrate, m_LatencySevenPacket * 1000);
    }
    break;
    }
}

void setpaddingts()
{
    static unsigned char NullPacket[188] = {0x47, 0x1F, 0xFE, 0x10, 'F', '5', 'O', 'E', 'O'};
    for (int i = 9; i < 188; i++)
        NullPacket[i] = i;
    pthread_mutex_lock(&buffer_mutexts);

    if(m_txmode == tx_dvbs2_ts)
        addneonts(NullPacket, /*7 **/ 188);
    if(m_txmode == tx_dvbs ) 
    {
        adddvbsts(NullPacket,188);
    }
    pthread_mutex_unlock(&buffer_mutexts);
}


void updatesdt(char *custom)
{
    FILE *cmd = popen("fw_printenv -n call", "r");
    char result[255] = {0x0};
    // fgets(result, sizeof(result), cmd);
    fscanf(cmd, "%s", result);
    if (strcmp(result, "") == 0)
        strcpy(result, "nocall");
    pclose(cmd);
    char sdt[512] = {0x0};

    if (custom)
    {
        sprintf(sdt, "%s%s", custom, result);
    }
    else
    {
        sprintf(sdt, "%s", result);
    }

    // fprintf(stderr,"SDT %s\n",sdt);
    char provider[255] = {0x0};
    sprintf(provider, "TezukaTV-%s(F5OEO)", COMIT_FW);
    int sum = 0;
    for (int i = 0; i < 10; i++)
    {
        sum += provider[i];
    }
    /*
    if (sum != 0x34F)
    {
        exit(1);
    }*/
    // fprintf(stderr,provider);
    customsdt = sdt_fmt(1, 1, 1, provider, sdt);

    char *byte = (char *)&customnullpacket[0];
    *byte++ = 0x47;
    *byte++ = 0x1F;
    *byte++ = 0xFF;
    *byte++ = 0x10;
    strcpy(byte, provider);
    byte += strlen(provider);
    char squeue[50];
    sprintf(squeue, "Queue:%d", m_bbframe_queue.size()); // Fixme ; this should be update regularly not only at start
    strcpy(byte, squeue);
    byte += strlen(squeue);
    sprintf(provider, "Comit:%s", COMIT_FW);
    strcpy(byte, provider);
    byte += strlen(provider);
    *byte++ = 0xFF;
}

#ifndef COMIT_FW
#define COMIT_FW "OUT_OF_TREE"
#endif
static pthread_t p_rxts;
void init_tsmux(char *mcast_ts, char *mcast_iface)
{

    updatesdt("");
    // DVBS2 INIT
    int status1 = dvbs2neon_control(0, CONTROL_RESET_FULL, (uint32)symbolbuff, sizeof(symbolbuff));
    int status2 = dvbs2neon_control(STREAM0, CONTROL_RESET_STREAM, 0, DATAMODE_TS);
    fmt.fec = 0;
    fmt.frame_type = FRAME_NORMAL;
    fmt.output_format = OUTPUT_FORMAT_BBFRAME;
    fmt.pilots = PILOTS_OFF;
    fmt.roll_off = RO_0_20;
    int status = dvbs2neon_control(STREAM0, CONTROL_SET_PARAMETERS, (uint32)&fmt, 0);

    // Seed the adaptive ModCod controller with the startup baseline
    { extern size_t m_SRtx; g_adaptiveModCod.init(fmt, (uint32_t)(m_SRtx / 4000)); }

    // DVBS INIT
    dvbsenco_init();
    viterbi_init(1); // 1/2

    // Other inits
    strcpy(m_mcast_ts, mcast_ts);
    strcpy(m_mcast_iface, mcast_iface);

    recv_ts_sock = udp_init(m_mcast_ts, m_mcast_iface, 1);
    build_crc8_table_r();
    pthread_mutex_init(&buffer_mutexts, NULL);
    pthread_create(&(p_rxts), NULL, &rx_ts_thread, NULL);
}