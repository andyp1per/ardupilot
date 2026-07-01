/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * RP2350 timer-driven statistical PC sampler. See rp2350_pc_sampler.h for the
 * mechanism and rationale.
 */

#include <ch.h>
#include "hal.h"

// Included after the ChibiOS headers so RP2350 and the enable macro (from
// hwdef.h via board.h) are defined when the header applies C linkage to the
// declarations below; otherwise the definitions would be C++-mangled and the
// extern "C" callers in ArduCopter would not resolve.
#include "rp2350_pc_sampler.h"

#if defined(RP2350) && defined(AP_RP2350_PC_SAMPLER_ENABLED)

/*
 * Sampling period in TIMER0 ticks (1 MHz). A prime, non-harmonic with the main
 * (~250 Hz), rate (~335 Hz) and EKF loops to avoid aliasing. ~5.08 kHz.
 */
#define PROF_INTERVAL_US   197u

/* 512-byte address buckets (fine enough to separate most functions). */
#define PROF_SHIFT         9u

/* Address regions that hold executable code, each mapped onto a contiguous
 * slice of the flat per-core histogram. Spans cover the linked extents with
 * headroom: flash .text (~1.42 MB), the __RAMFUNC2__ SRAM block, and the two
 * 4 KB scratch banks used by the scratchx/scratchy registries. */
static const struct {
    uint32_t base;
    uint32_t nbuckets;
    uint32_t off;
    char     tag;
} prof_regions[] = {
    { 0x10010000u, 2816u, 0u,           'F' },  /* XIP flash text            */
    { 0x20000000u,  256u, 2816u,        'S' },  /* SRAM ramfunc              */
    { 0x20080000u,   16u, 2816u + 256u, 'C' },  /* scratch banks (ram4/ram5) */
};
#define PROF_NREGIONS   (sizeof(prof_regions) / sizeof(prof_regions[0]))
#define PROF_NBUCKETS   (2816u + 256u + 16u)

/* Valid process-stack range for the PSP guard (ram0 plus scratch banks). */
#define PROF_PSP_MIN    0x20000000u
#define PROF_PSP_MAX    0x20081fe0u

#define PROF_DUMP_MAX   16u

struct prof_core {
    uint16_t hist[PROF_NBUCKETS];  /* saturating per-bucket sample counts */
    uint32_t total;                /* samples with a valid stacked PC     */
    uint32_t other;                /* PC outside every code region        */
    uint32_t bad;                  /* PSP outside the valid stack range   */
};

static struct prof_core prof[2];
static bool inited_core0;
static bool inited_core1;

/*
 * Alarm callback, invoked from the ST LLD TIMER0 ALARMn handler (ALARM2 on
 * core0, ALARM3 on core1) after it has acknowledged the interrupt. Reads the
 * interrupted thread's PC from the exception frame (frame word 6 = PSP+0x18,
 * a fixed offset even with FPU lazy stacking) and buckets it, then re-arms.
 *
 * Placed in SRAM (.ramtext) so this ~5 kHz ISR never stalls on an XIP cache
 * miss; it only uses inlined intrinsics and memory-mapped registers, so it
 * makes no calls into flash.
 */
__attribute__((noinline, section(".ramtext")))
static void pc_sampler_cb(unsigned alarm)
{
    /* Schedule the next sample. INTE stays set; the LLD already cleared INTR. */
    TIMER0->ALARM[alarm] = TIMER0->TIMERAWL + PROF_INTERVAL_US;

    const unsigned core = (unsigned)port_get_core_id() & 1u;
    struct prof_core *pc = &prof[core];

    const uint32_t psp = __get_PSP();
    if (psp < PROF_PSP_MIN || psp > PROF_PSP_MAX) {
        pc->bad++;
        return;
    }

    const uint32_t addr = ((const uint32_t *)psp)[6];
    pc->total++;
    for (unsigned r = 0; r < PROF_NREGIONS; r++) {
        const uint32_t rel = addr - prof_regions[r].base;
        if (rel < (prof_regions[r].nbuckets << PROF_SHIFT)) {
            uint16_t *b = &pc->hist[prof_regions[r].off + (rel >> PROF_SHIFT)];
            if (*b != 0xFFFFu) {
                (*b)++;
            }
            return;
        }
    }
    pc->other++;
}

/* Arm one free alarm and enable its IRQ on the calling core. */
static void sampler_arm(unsigned alarm)
{
    stSetCallback(alarm, pc_sampler_cb);
    TIMER0->INTR = (1u << alarm);                          /* drop any stale latch */
    TIMER0->ALARM[alarm] = TIMER0->TIMERAWL + PROF_INTERVAL_US;
    TIMER0->SET.INTE = (1u << alarm);                      /* atomic, no RMW race  */
}

void rp2350_pc_sampler_init_core0(void)
{
    if (inited_core0) {
        return;
    }
    inited_core0 = true;
    sampler_arm(2);
    nvicEnableVector(RP_TIMER0_IRQ2_NUMBER, RP_IRQ_TIMER0_ALARM2_PRIORITY);
}

void rp2350_pc_sampler_init_core1(void)
{
    if (inited_core1) {
        return;
    }
    inited_core1 = true;
    sampler_arm(3);
    nvicEnableVector(RP_TIMER0_IRQ3_NUMBER, RP_IRQ_TIMER0_ALARM3_PRIORITY);
}

/* Map a flat bucket index back to its region tag and byte offset from the
 * region base. */
static char bucket_addr(unsigned idx, uint32_t *off)
{
    for (unsigned r = 0; r < PROF_NREGIONS; r++) {
        if (idx - prof_regions[r].off < prof_regions[r].nbuckets) {
            *off = (idx - prof_regions[r].off) << PROF_SHIFT;
            return prof_regions[r].tag;
        }
    }
    *off = 0;
    return '?';
}

/* Minimal append helpers - keep one byte spare for the terminating NUL. */
static uint32_t put_str(char *b, uint32_t u, uint32_t cap, const char *s)
{
    while (*s != '\0' && u + 1u < cap) {
        b[u++] = *s++;
    }
    return u;
}
static uint32_t put_ch(char *b, uint32_t u, uint32_t cap, char c)
{
    if (u + 1u < cap) {
        b[u++] = c;
    }
    return u;
}
static uint32_t put_dec(char *b, uint32_t u, uint32_t cap, uint32_t v)
{
    char t[10];
    unsigned n = 0;
    do {
        t[n++] = (char)('0' + v % 10u);
        v /= 10u;
    } while (v != 0 && n < sizeof(t));
    while (n != 0 && u + 1u < cap) {
        b[u++] = t[--n];
    }
    return u;
}
static uint32_t put_hex(char *b, uint32_t u, uint32_t cap, uint32_t v)
{
    char t[8];
    unsigned n = 0;
    do {
        const unsigned d = v & 0xFu;
        t[n++] = (char)(d < 10u ? '0' + d : 'a' + d - 10u);
        v >>= 4;
    } while (v != 0 && n < sizeof(t));
    while (n != 0 && u + 1u < cap) {
        b[u++] = t[--n];
    }
    return u;
}

uint32_t rp2350_pc_sampler_dump(unsigned core, unsigned maxn,
                                char *buf, uint32_t buflen)
{
    if (core > 1 || buf == nullptr || buflen == 0) {
        return 0;
    }
    if (maxn > PROF_DUMP_MAX) {
        maxn = PROF_DUMP_MAX;
    }
    const struct prof_core *pc = &prof[core];

    /* Single pass: per-region sample sums and a descending top-N list. */
    uint32_t rsum[PROF_NREGIONS] = {};
    uint16_t top_idx[PROF_DUMP_MAX];
    uint16_t top_cnt[PROF_DUMP_MAX];
    unsigned ntop = 0;
    for (unsigned i = 0; i < PROF_NBUCKETS; i++) {
        const uint16_t c = pc->hist[i];
        if (c == 0) {
            continue;
        }
        for (unsigned r = 0; r < PROF_NREGIONS; r++) {
            if (i - prof_regions[r].off < prof_regions[r].nbuckets) {
                rsum[r] += c;
                break;
            }
        }
        if (ntop < maxn || (maxn > 0 && c > top_cnt[maxn - 1])) {
            unsigned j = (ntop < maxn) ? ntop++ : (maxn - 1);
            while (j > 0 && top_cnt[j - 1] < c) {
                top_cnt[j] = top_cnt[j - 1];
                top_idx[j] = top_idx[j - 1];
                j--;
            }
            top_cnt[j] = c;
            top_idx[j] = (uint16_t)i;
        }
    }

    const uint32_t total = pc->total ? pc->total : 1;
    uint32_t used = 0;
    used = put_str(buf, used, buflen, "n=");
    used = put_dec(buf, used, buflen, pc->total);
    used = put_str(buf, used, buflen, " F=");
    used = put_dec(buf, used, buflen, (uint32_t)((uint64_t)rsum[0] * 100u / total));
    used = put_str(buf, used, buflen, "% S=");
    used = put_dec(buf, used, buflen, (uint32_t)((uint64_t)rsum[1] * 100u / total));
    used = put_str(buf, used, buflen, "% o=");
    used = put_dec(buf, used, buflen, (uint32_t)((uint64_t)pc->other * 100u / total));
    used = put_ch(buf, used, buflen, '%');

    /* Hot buckets as region-tagged offset:count tokens, wrapped near 40 chars
     * so each line fits a MAVLink STATUSTEXT. */
    uint32_t line = 999;  /* force a newline before the first token */
    for (unsigned k = 0; k < ntop && used + 18u < buflen; k++) {
        uint32_t off = 0;
        const char tag = bucket_addr(top_idx[k], &off);
        if (line >= 40) {
            used = put_ch(buf, used, buflen, '\n');
            line = 0;
        } else {
            used = put_ch(buf, used, buflen, ' ');
            line += 1;
        }
        const uint32_t start = used;
        used = put_ch(buf, used, buflen, tag);
        used = put_hex(buf, used, buflen, off);
        used = put_ch(buf, used, buflen, ':');
        used = put_dec(buf, used, buflen, top_cnt[k]);
        line += used - start;
    }
    buf[used] = '\0';
    return used;
}

#endif /* RP2350 && AP_RP2350_PC_SAMPLER_ENABLED */
