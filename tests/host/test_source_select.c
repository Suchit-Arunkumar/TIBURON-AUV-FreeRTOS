/*
 * Host test for the preferred/backup source selection
 * (Core/Src/source_select.c), stepped through time in 10 ms ticks.
 */
#include "test.h"
#include "source_select.h"
#include "sensor_types.h"

static SourceSelect s;
static uint32_t now;
static int a_on, b_on;              /* is each sensor producing readings */
static uint32_t a_last, b_last;
static int a_seen, b_seen;

/* Advance ms milliseconds, sensors reporting every 10 ms while on. */
static uint8_t run(uint32_t ms)
{
    uint8_t act = SRC_NONE;

    for (uint32_t t = 0; t < ms; t += 10)
    {
        now += 10;
        if (a_on) { a_last = now; a_seen = 1; }
        if (b_on) { b_last = now; b_seen = 1; }
        act = source_select_update(&s, now, a_seen, a_last, b_seen, b_last);
    }

    return act;
}

int main(void)
{
    source_select_init(&s, SRC_VN200, SRC_BNO085);
    now = 1000;

    /* Nothing yet. */
    CHECK(run(50) == SRC_NONE);

    /* Only the backup at boot: used straight away. */
    b_on = 1;
    CHECK(run(50) == SRC_BNO085);

    /* Preferred appears: not taken until it has been fresh for 1 s. */
    a_on = 1;
    CHECK(run(500) == SRC_BNO085);
    CHECK(run(600) == SRC_VN200);

    /* Preferred drops out: back to the backup once it is 200 ms stale. */
    a_on = 0;
    CHECK(run(100) == SRC_VN200);
    CHECK(run(200) == SRC_BNO085);

    /* Preferred flickers on and off every 300 ms: never switched back. */
    for (int i = 0; i < 5; i++)
    {
        a_on = 1; CHECK(run(300) == SRC_BNO085);
        a_on = 0; CHECK(run(300) == SRC_BNO085);
    }

    /* Both lost: none. */
    b_on = 0;
    CHECK(run(300) == SRC_NONE);

    /* Preferred alone comes back: taken at once (nothing else to use). */
    a_on = 1;
    CHECK(run(20) == SRC_VN200);

    /* Backup lost while on the backup, preferred fresh but not settled:
     * take the preferred rather than nothing. */
    source_select_init(&s, SRC_VN200, SRC_BNO085);
    a_on = 0; b_on = 1;
    run(300);
    CHECK(run(10) == SRC_BNO085);
    a_on = 1; run(100);
    b_on = 0;
    CHECK(run(300) == SRC_VN200);

    /* Tick counter wrapping through zero does not upset staleness. */
    source_select_init(&s, SRC_BAR30, SRC_ADC);
    now = 0xFFFFFF00u;
    a_on = 1; b_on = 0; a_seen = b_seen = 0;
    CHECK(run(500) == SRC_BAR30);

    return test_summary("source_sel");
}
